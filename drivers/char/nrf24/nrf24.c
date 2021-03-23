#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>

#define DEVICE_NAME "nrf24-device"
#define NRF24_CE 		16								// GPIO16
#define NRF24_SCLK	20								// GPIO20
#define NRF24_MOSI	21								// GPIO21
#define NRF24_CSN		26								// GPIO26

#define NRF24_IRQ	 	12								// GPIO12
#define NRF24_MISO	19								// GPIO19

#define NRF24_COMMAND_NOP 0xff

#define HALF_PERIOD	500								// SCLK half period

/* Per device structure */
struct nrf24_dev {
	char data[4];
	struct cdev cdev;
	char name[10];
	int busy;
	struct mutex lock;   
} * nrf24_devp;

int irq_counter;

static ssize_t nrf24_read(struct file*, char*, size_t, loff_t*);
static ssize_t nrf24_write(struct file *, const char __user *, size_t, loff_t *);
static int nrf24_open(struct inode *, struct file *);
static int nrf24_release(struct inode *, struct file *);
static u8 nrf24_send_byte(u8 value);

static struct file_operations nrf24_fops = {
	.owner = THIS_MODULE,	/* Owner */
	.read = nrf24_read,
	.write = nrf24_write,
	.open = nrf24_open,
	.release = nrf24_release
};

static dev_t nrf24_device_number;
struct class* nrf24_class;


static irqreturn_t nrf24_irq_handler(int irq, void* dev_id)
{
	irq_counter++;
	printk(KERN_INFO "nrf24 %i irq handled: %i \n", NRF24_IRQ, irq_counter);
	return IRQ_HANDLED;
}

static int __init nrf24_mod_init(void)
{
	int irq_line;

	// allocate chrdev region

	if (alloc_chrdev_region(&nrf24_device_number, 0, 1, DEVICE_NAME))
	{
		printk(KERN_INFO "Cannot allocate region for nrf24 device");
		return -1;
	}

	// populate sysfs entries 
	nrf24_class = class_create(THIS_MODULE, DEVICE_NAME);
	
	// allocate memory for device structure
	nrf24_devp = kmalloc(sizeof(struct nrf24_dev), GFP_KERNEL);	
;
	if (!nrf24_devp)
	{
		printk(KERN_INFO "Bad kmalloc\n");
		return -1;
	}
	mutex_init(&nrf24_devp->lock);

	if (mutex_lock_interruptible(&nrf24_devp->lock))	
	{
		printk(KERN_INFO "nrf24 init interrupted \n");
		return -1;
	}
	
	// connect fops with cdev
	cdev_init(&nrf24_devp->cdev, &nrf24_fops);
	nrf24_devp->cdev.owner = THIS_MODULE;
	
	// connect minor major number to cdev
 	if (cdev_add(&nrf24_devp->cdev, nrf24_device_number, 1)) {
		printk(KERN_INFO "cdev add failed\n");
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}
	
	// send uevent to udev
	device_create(nrf24_class, NULL, nrf24_device_number, NULL, "nrf24d");
	
	// setup output pins	
	nrf24_devp->busy=0;
	nrf24_devp->data[0]='0';
	nrf24_devp->data[1]='0';
	nrf24_devp->data[2]='0';
	nrf24_devp->data[3]='0';

	// set IRQ and MISO to input

	if (gpio_direction_input(NRF24_IRQ)!=0 ||
		gpio_direction_input(NRF24_MISO)!=0)
	{
		printk(KERN_INFO "Cannot set GPIO %i %i to input \n", NRF24_IRQ, NRF24_MISO);
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}

	if (gpio_direction_output(NRF24_CE,1)!=0 ||
		gpio_direction_output(NRF24_SCLK,1)!=0 ||
		gpio_direction_output(NRF24_MOSI,1)!=0 ||
		gpio_direction_output(NRF24_CSN,1)!=0)

	{
		printk(KERN_INFO "Cannot set GPIO %i %i %i %i to output \n", NRF24_CE, NRF24_SCLK, NRF24_MOSI, NRF24_CSN);
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}

	// request irq for GPIO 12

	irq_line = gpio_to_irq(NRF24_IRQ);
	printk(KERN_INFO "IRQ line for GPIO %i is %i \n", NRF24_IRQ, irq_line);
	
	if (request_irq(irq_line, nrf24_irq_handler, IRQF_TRIGGER_FALLING, "Interrupt nRF24 GPIO ", NULL)<0)
	{
		printk(KERN_INFO "Cannot get IRQ for GPIO %i \n", NRF24_IRQ);
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}
	
	irq_counter = 0;
	printk(KERN_INFO "nrf24 device initialized\n");

	mutex_unlock(&nrf24_devp->lock);

	gpio_set_value(NRF24_CE, 0);
	gpio_set_value(NRF24_CSN, 1);
	gpio_set_value(NRF24_SCLK, 0);
	return 0;
}

static void __exit nrf24_mod_exit(void)
{
	// release irq line       
	int irq_line;
	irq_line = gpio_to_irq(NRF24_IRQ);
	free_irq(irq_line, NULL);

	// remove cdev
	cdev_del(&nrf24_devp->cdev);

	// release major number
	unregister_chrdev_region(nrf24_device_number, 1);

	// destroy device
	device_destroy(nrf24_class, nrf24_device_number);

	// destroy cmos class
	class_destroy(nrf24_class);
	printk(KERN_INFO "nrf24 exit done\n");

	kfree(nrf24_devp);

	return;
}

static ssize_t nrf24_read(struct file *file, char* buf, size_t count, loff_t * offset)
{
	struct nrf24_dev* nrf24_devp;
	printk(KERN_INFO "nrf24 read started \n");
	nrf24_devp=file->private_data;
	
	if (mutex_lock_interruptible(&nrf24_devp->lock))	
	{
		printk(KERN_INFO "nrf24 init interrupted \n");
		return -1;
	} 

	nrf24_devp->data[0] = gpio_get_value(16)+48;
	nrf24_devp->data[1] = gpio_get_value(20)+48;
	nrf24_devp->data[2] = gpio_get_value(21)+48;
	nrf24_devp->data[3] = gpio_get_value(26)+48;

	if (copy_to_user(buf, (void*)nrf24_devp->data, 3)!=0)
	{
		return -EIO;
	}
	printk(KERN_INFO "nrf24 count=%i\n",count);
	if (nrf24_devp->busy) {
		nrf24_devp->busy=0;
		return 0;
	} else
	{
		nrf24_devp->busy=1;
		return 2;
	}	
	mutex_unlock(&nrf24_devp->lock);
	return 2;
}

static ssize_t nrf24_write(struct file * filep, const char __user * userp, size_t size, loff_t * offset)
{
	int i;
	u8 ret=0;
	char kbuf[4];
	if (copy_from_user(kbuf, userp, 4)!=0)
		return -1;
		
	if (mutex_lock_interruptible(&nrf24_devp->lock))	
	{
		printk(KERN_INFO "gpio init interrupted \n");
		return -1;
	} 

	nrf24_devp=filep->private_data;
	for (i=0; i<4; i++)
		nrf24_devp->data[i] = kbuf[i];

	printk(KERN_INFO "kbuf[0] %i \n", kbuf[0]);	
	//gpio_set_value(16,(int)(kbuf[0]-48));
	//gpio_set_value(20,(int)(kbuf[1]-48));
	//gpio_set_value(21,(int)(kbuf[2]-48));
	//gpio_set_value(26,(int)(kbuf[3]-48));

	gpio_set_value(NRF24_CSN, 0);
	ndelay(HALF_PERIOD);
	ndelay(HALF_PERIOD);

	nrf24_send_byte(kbuf[0]);
	ret = nrf24_send_byte(NRF24_COMMAND_NOP);
	gpio_set_value(NRF24_CSN, 1);
	printk(KERN_INFO "MISO: %i\n", ret);

	mutex_unlock(&nrf24_devp->lock);
	return 2;
}

static int nrf24_open(struct inode * node, struct file * file)
{
	struct nrf24_dev *nrf24_devp;
	nrf24_devp = container_of(node->i_cdev, struct nrf24_dev, cdev);
	file->private_data = nrf24_devp;
	printk(KERN_INFO "nrf24 open done\n");

	return 0;
}

static int nrf24_release(struct inode * node, struct file * file)
{
	printk(KERN_INFO "nrf24 release done\n");
	return 0;
}

static u8 nrf24_send_byte(u8 value)
{
	int i=0;
	u8 ret=0;
	printk(KERN_INFO "value %i \n", value);

	for(i=7;i>=0;i--)
	{
		if (value & (1 << i))
		{
			gpio_set_value(NRF24_MOSI, 1);
		} else
		{
			gpio_set_value(NRF24_MOSI, 0);
		}

		gpio_set_value(NRF24_SCLK, 0);
		ndelay(HALF_PERIOD);
	
		printk(KERN_INFO "clk %i \n", gpio_get_value(NRF24_MISO));
		ret = ret | (gpio_get_value(NRF24_MISO) << i); 
		gpio_set_value(NRF24_SCLK, 1);
		ndelay(HALF_PERIOD);
	}

	gpio_set_value(NRF24_SCLK, 0);
	return ret;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("bhorvath");
MODULE_DESCRIPTION("nrf24 cdev module");

module_init(nrf24_mod_init);
module_exit(nrf24_mod_exit);
 
