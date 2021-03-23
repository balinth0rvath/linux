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

// GPIO pin settings 
#define NRF24_GPIO_CE 								16								// GPIO16
#define NRF24_GPIO_SCLK								20								// GPIO20
#define NRF24_GPIO_MOSI								21								// GPIO21
#define NRF24_GPIO_CSN								26								// GPIO26
#define NRF24_GPIO_IRQ	 							12								// GPIO12
#define NRF24_GPIO_MISO								19								// GPIO19
#define NRF24_SPI_HALF_CLK						500								// SCLK half period ns

// nRF24 commands
#define NRF24_CMD_R_REGISTER 					0x00
#define NRF24_CMD_W_REGISTER 					0x20
#define NRF24_CMD_R_RX_PAYLOAD				0x61
#define NRF24_CMD_W_TX_PAYLOAD				0xa0
#define NRF24_CMD_FLUSH_TX 						0xe1
#define NRF24_CMD_FLUSH_RX 						0xe2
#define NRF24_CMD_REUSE_TX_PL 				0xe3
#define NRF24_CMD_ACTIVATE						0x50
#define NRF24_CMD_R_RX_PL_WID 				0x60
#define NRF24_CMD_W_ACK_PAYLOAD 			0xa8
#define NRF24_CMD_W_TX_PAYLOAD_NO_ACK	0xb0
#define NRF24_CMD_NOP 								0xff

// nRF24 registers
#define NRF24_REG_CONFIG							0x00
#define NRF24_REG_EN_AA								0x01
#define NRF24_REG_EN_RXADDR						0x02
#define NRF24_REG_SETUP_AW						0x03
#define NRF24_REG_SETUP_RETR					0x04
#define NRF24_REG_RF_CH								0x05
#define NRF24_REG_RF_SETUP						0x06
#define NRF24_REG_STATUS							0x07
#define NRF24_REG_OBSERVE_TX					0x08
#define NRF24_REG_CD									0x09
#define NRF24_REG_RX_ADDR_P0					0x0a
#define NRF24_REG_RX_ADDR_P1					0x0b
#define NRF24_REG_RX_ADDR_P2					0x0c
#define NRF24_REG_RX_ADDR_P3					0x0d
#define NRF24_REG_RX_ADDR_P4					0x0e
#define NRF24_REG_RX_ADDR_P5					0x0f
#define NRF24_REG_TX_ADDR							0x10
#define NRF24_REG_RX_PW_P0						0x11
#define NRF24_REG_RX_PW_P1						0x12
#define NRF24_REG_RX_PW_P2						0x13
#define NRF24_REG_RX_PW_P3						0x14
#define NRF24_REG_RX_PW_P4						0x15
#define NRF24_REG_RX_PW_P5						0x16
#define NRF24_REG_FIFO_STATUS					0x17
#define NRF24_REG_DYNPD								0x1c
#define NRF24_REG_FEATURE							0x1d

#define NRF24_REG_STATUS_DEFAULT			0x0e

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

static int nrf24_check_device(void);

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
	printk(KERN_INFO "nrf24 %i irq handled: %i \n", NRF24_GPIO_IRQ, irq_counter);
	return IRQ_HANDLED;
}

static int __init nrf24_mod_init(void)
{
	int irq_line;

	printk(KERN_INFO "Start initializing nRF24 device... \n");
	if (nrf24_check_device())
	{
		printk(KERN_INFO "nRF24 device not found \n");
		return -1;
	}	
	
	// allocate chrdev region

	if (alloc_chrdev_region(&nrf24_device_number, 0, 1, DEVICE_NAME))
	{
		printk(KERN_INFO "Cannot allocate region for nrf24 device \n");
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

	if (gpio_direction_input(NRF24_GPIO_IRQ)!=0 ||
		gpio_direction_input(NRF24_GPIO_MISO)!=0)
	{
		printk(KERN_INFO "Cannot set GPIO %i %i to input \n", NRF24_GPIO_IRQ, NRF24_GPIO_MISO);
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}

	if (gpio_direction_output(NRF24_GPIO_CE,1)!=0 ||
		gpio_direction_output(NRF24_GPIO_SCLK,1)!=0 ||
		gpio_direction_output(NRF24_GPIO_MOSI,1)!=0 ||
		gpio_direction_output(NRF24_GPIO_CSN,1)!=0)

	{
		printk(KERN_INFO "Cannot set GPIO %i %i %i %i to output \n", NRF24_GPIO_CE, NRF24_GPIO_SCLK, NRF24_GPIO_MOSI, NRF24_GPIO_CSN);
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}

	// request irq for GPIO 12

	irq_line = gpio_to_irq(NRF24_GPIO_IRQ);
	printk(KERN_INFO "IRQ line for GPIO %i is %i \n", NRF24_GPIO_IRQ, irq_line);
	
	if (request_irq(irq_line, nrf24_irq_handler, IRQF_TRIGGER_FALLING, "Interrupt nRF24 GPIO ", NULL)<0)
	{
		printk(KERN_INFO "Cannot get IRQ for GPIO %i \n", NRF24_GPIO_IRQ);
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}
	
	irq_counter = 0;
	printk(KERN_INFO "nrf24 device initialized\n");

	mutex_unlock(&nrf24_devp->lock);

	gpio_set_value(NRF24_GPIO_CE, 0);
	gpio_set_value(NRF24_GPIO_CSN, 1);
	gpio_set_value(NRF24_GPIO_SCLK, 0);
	return 0;
}

static void __exit nrf24_mod_exit(void)
{
	// release irq line       
	int irq_line;
	irq_line = gpio_to_irq(NRF24_GPIO_IRQ);
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

	gpio_set_value(NRF24_GPIO_CSN, 0);
	ndelay(NRF24_SPI_HALF_CLK);
	ndelay(NRF24_SPI_HALF_CLK);

	nrf24_send_byte(kbuf[0]);
	ret = nrf24_send_byte(NRF24_CMD_NOP);
	gpio_set_value(NRF24_GPIO_CSN, 1);
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

static int nrf24_check_device()
{
	u8 ret = 0;
	gpio_set_value(NRF24_GPIO_CSN, 0);
	ndelay(NRF24_SPI_HALF_CLK);

	nrf24_send_byte(NRF24_CMD_R_REGISTER | NRF24_REG_STATUS);
	ret = nrf24_send_byte(NRF24_CMD_NOP);
	gpio_set_value(NRF24_GPIO_CSN, 1);
	return (ret != NRF24_REG_STATUS_DEFAULT);
}

static u8 nrf24_send_byte(u8 value)
{
	int i=0;
	u8 ret=0;

	for(i=7;i>=0;i--)
	{
		if (value & (1 << i))
		{
			gpio_set_value(NRF24_GPIO_MOSI, 1);
		} else
		{
			gpio_set_value(NRF24_GPIO_MOSI, 0);
		}

		gpio_set_value(NRF24_GPIO_SCLK, 0);
		ndelay(NRF24_SPI_HALF_CLK);
	
		ret = ret | (gpio_get_value(NRF24_GPIO_MISO) << i); 
		gpio_set_value(NRF24_GPIO_SCLK, 1);
		ndelay(NRF24_SPI_HALF_CLK);
	}

	gpio_set_value(NRF24_GPIO_SCLK, 0);
	return ret;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("bhorvath");
MODULE_DESCRIPTION("nrf24 cdev module");

module_init(nrf24_mod_init);
module_exit(nrf24_mod_exit);
 
