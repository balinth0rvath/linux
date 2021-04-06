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
#include <linux/nrf24.h>

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
	char payload_buffer[32];
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
static long nrf24_ioctl(struct file*, unsigned int, unsigned long);

static void nrf24_init_device(void);
static void nrf24_show_status(void);
static int 	nrf24_check_device(void);
static int nrf24_get_register(u8 reg);
static void nrf24_get_address_register(u8 reg, u8* result);
static void nrf24_write_register(u8 register, u8 value, u8 mask);
static u8 	nrf24_send_byte(u8 value);

static struct file_operations nrf24_fops = {
	.owner = THIS_MODULE,	/* Owner */
	.read = nrf24_read,
	.write = nrf24_write,
	.open = nrf24_open,
	.release = nrf24_release,
	.unlocked_ioctl = nrf24_ioctl
};

static dev_t nrf24_device_number;
struct class* nrf24_class;

static irqreturn_t nrf24_irq_handler(int irq, void* dev_id)
{
	irq_counter++;
	printk(KERN_INFO "nrf24: %i irq handled: %i \n", NRF24_GPIO_IRQ, irq_counter);
	return IRQ_HANDLED;
}

static int __init nrf24_mod_init(void)
{
	int irq_line;
	int i;
	printk(KERN_INFO " ------------ \nnrf24: Start initializing nRF24 device... \n");

	// allocate chrdev region
	if (alloc_chrdev_region(&nrf24_device_number, 0, 1, DEVICE_NAME))
	{
		printk(KERN_INFO "nrf24: Cannot allocate region for nrf24 device \n");
		return -1;
	}

	// populate sysfs entries 
	nrf24_class = class_create(THIS_MODULE, DEVICE_NAME);
	
	// allocate memory for device structure
	nrf24_devp = kmalloc(sizeof(struct nrf24_dev), GFP_KERNEL);	
;
	if (!nrf24_devp)
	{
		printk(KERN_INFO "nrf24: Bad kmalloc\n");
		return -1;
	}
	mutex_init(&nrf24_devp->lock);

	if (mutex_lock_interruptible(&nrf24_devp->lock))	
	{
		printk(KERN_INFO "nrf24: Init interrupted \n");
		return -1;
	}
	
	// connect fops with cdev
	cdev_init(&nrf24_devp->cdev, &nrf24_fops);
	nrf24_devp->cdev.owner = THIS_MODULE;
	
	// connect minor major number to cdev
 	if (cdev_add(&nrf24_devp->cdev, nrf24_device_number, 1)) {
		printk(KERN_INFO "nrf24: cdev add failed\n");
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}
	
	// send uevent to udev
	device_create(nrf24_class, NULL, nrf24_device_number, NULL, "nrf24d");
	
	// clear payload buffer	
	nrf24_devp->busy=0;
	for(i=0;i<32;i++)
		nrf24_devp->payload_buffer[i]='0';

	// set IRQ and MISO to input

	if (gpio_direction_input(NRF24_GPIO_IRQ)!=0 ||
		gpio_direction_input(NRF24_GPIO_MISO)!=0)
	{
		printk(KERN_INFO "nrf24: Cannot set GPIO %i %i to input \n", NRF24_GPIO_IRQ, NRF24_GPIO_MISO);
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}

	if (gpio_direction_output(NRF24_GPIO_CE,1)!=0 ||
		gpio_direction_output(NRF24_GPIO_SCLK,1)!=0 ||
		gpio_direction_output(NRF24_GPIO_MOSI,1)!=0 ||
		gpio_direction_output(NRF24_GPIO_CSN,1)!=0)

	{
		printk(KERN_INFO "nrf24: Cannot set GPIO %i %i %i %i to output \n", NRF24_GPIO_CE, NRF24_GPIO_SCLK, NRF24_GPIO_MOSI, NRF24_GPIO_CSN);
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}

	gpio_set_value(NRF24_GPIO_CE, 0);
	gpio_set_value(NRF24_GPIO_CSN, 1);
	gpio_set_value(NRF24_GPIO_SCLK, 0);
	if (nrf24_check_device())
	{
		printk(KERN_INFO "nrf24: Device not found \n");
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}	

	// request irq for GPIO 12
	irq_line = gpio_to_irq(NRF24_GPIO_IRQ);
	printk(KERN_INFO "nrf24: IRQ line for GPIO %i is %i \n", NRF24_GPIO_IRQ, irq_line);
	
	if (request_irq(irq_line, nrf24_irq_handler, IRQF_TRIGGER_FALLING, "Interrupt nRF24 GPIO ", NULL)<0)
	{
		printk(KERN_INFO "nrf24: Cannot get IRQ for GPIO %i \n", NRF24_GPIO_IRQ);
		mutex_unlock(&nrf24_devp->lock);
		return -1;
	}
	irq_counter = 0;

	printk(KERN_INFO "nrf24: reset status:\n");
	nrf24_show_status();
	nrf24_init_device();
	printk(KERN_INFO "nrf24: Device initialized\n");
	nrf24_show_status();
	mutex_unlock(&nrf24_devp->lock);
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
	printk(KERN_INFO "nrf24: Exit done\n");

	kfree(nrf24_devp);

	return;
}

static ssize_t nrf24_read(struct file *file, char* buf, size_t count, loff_t * offset)
{
	struct nrf24_dev* nrf24_devp;
	nrf24_devp=file->private_data;
	
	if (mutex_lock_interruptible(&nrf24_devp->lock))	
	{
		printk(KERN_INFO "nrf24: Init interrupted \n");
		return -1;
	} 


	if (copy_to_user(buf, (void*)nrf24_devp->payload_buffer, 3)!=0)
	{
		return -EIO;
	}
	printk(KERN_INFO "nrf24: Count=%i\n",count);
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
	u8 status;
	int wait = 1000;
	char payload[4];
	if (copy_from_user(payload, userp, 4)!=0)
		return -1;
		
	if (mutex_lock_interruptible(&nrf24_devp->lock))	
	{
		printk(KERN_INFO "nrf24: Error, cannot lock device data \n");
		return -1;
	} 

	nrf24_devp=filep->private_data;
	for (i=0; i<4; i++)
		nrf24_devp->payload_buffer[i] = payload[i];

	printk(KERN_INFO "nrf24: Transmitting payload %i %i %i %i \n", 
		payload[0], payload[1], payload[2], payload[3]);	

	gpio_set_value(NRF24_GPIO_CE, 0);
	gpio_set_value(NRF24_GPIO_CSN, 0);
	ndelay(NRF24_SPI_HALF_CLK);
	ndelay(NRF24_SPI_HALF_CLK);

	nrf24_send_byte(NRF24_CMD_W_TX_PAYLOAD);
	for(i=0; i<4; i++)
		nrf24_send_byte(payload[i]);

	gpio_set_value(NRF24_GPIO_CSN, 1);
	gpio_set_value(NRF24_GPIO_CE, 1);

	do {
		status = nrf24_get_register(NRF24_REG_STATUS);
		if (status & 0x30)
			break;
		mdelay(1);
	} while (wait--);


	printk(KERN_INFO "nrf24: status %i \n", status);
	printk(KERN_INFO "nrf24: wait   %i \n", wait);
	gpio_set_value(NRF24_GPIO_CE, 0);

	if (!wait)
	{
		printk(KERN_INFO "nrf24: Transmit error, timeout \n");
	}

	if (status & 0x10)
	{
		printk(KERN_INFO "nrf24: Transmit error, max retry count reached \n");
		nrf24_write_register(NRF24_REG_STATUS, 0x30, 0x30);
	} else 
	if (status & 0x20)
	{
		printk(KERN_INFO "nrf24: Transmit success \n");

	} else
		printk(KERN_INFO "nrf24: Transmit error \n");
	mutex_unlock(&nrf24_devp->lock);
	printk(KERN_INFO "nrf24: MAX_RT,TX_DS cleared \n");
	nrf24_write_register(NRF24_REG_STATUS, 0x30, 0x30);
	return 5;
}

static int nrf24_open(struct inode * node, struct file * file)
{
	struct nrf24_dev *nrf24_devp;
	nrf24_devp = container_of(node->i_cdev, struct nrf24_dev, cdev);
	file->private_data = nrf24_devp;
	printk(KERN_INFO "nrf24: Open done\n");

	return 0;
}
static int nrf24_release(struct inode * node, struct file * file)
{
	printk(KERN_INFO "nrf24: Release done\n");
	return 0;
}

static long nrf24_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
	//u8 ret = 0;
	printk(KERN_INFO "nrf24: ioctl command: %iu argument %lu \n", cmd, arg);
	switch (cmd)
	{
		case NRF24_SET_PRIM_RX:
			nrf24_write_register(NRF24_REG_CONFIG, 1, 1); 
			
			break;

		case NRF24_SET_PRIM_TX:
			nrf24_write_register(NRF24_REG_CONFIG, 0, 1);
			break;
		
		default:
			break;
	}

	return 0;
}

static void nrf24_show_status()
{
	u8 regvalue;
	u8 result[5];
	printk(KERN_INFO "nrf24: register summary \n");

	regvalue = nrf24_get_register(NRF24_REG_CONFIG);
	if (regvalue & 0x1)
		printk(KERN_INFO "nrf24: Primary RX \n");
	else
		printk(KERN_INFO "nrf24: Primary TX \n");
	if (regvalue & 0x2)
		printk(KERN_INFO "nrf24: Power up \n");
	else
		printk(KERN_INFO "nrf24: Power down \n");
	if (regvalue & 0x8)	
		printk(KERN_INFO "nrf24: CRC enabled \n");
	else
		printk(KERN_INFO "nrf24: CRC disabled \n");

	if (regvalue & 0x4)	
		printk(KERN_INFO "nrf24: CRC 2 bytes \n");
	else
		printk(KERN_INFO "nrf24: CRC 1 byte \n");

	regvalue = nrf24_get_register(NRF24_REG_EN_AA);
	printk(KERN_INFO "nrf24: Auto acknowledgement enabled: %x \n", regvalue);

	regvalue = nrf24_get_register(NRF24_REG_EN_RXADDR);
	printk(KERN_INFO "nrf24: Enabled data pipes: %x \n", regvalue);

	regvalue = nrf24_get_register(NRF24_REG_SETUP_AW);
	printk(KERN_INFO "nrf24: Address width %i bytes \n", regvalue + 2);

	regvalue = nrf24_get_register(NRF24_REG_SETUP_RETR);
	printk(KERN_INFO "nrf24: Auto retransmit delay: %i, count: %i \n", 
		((regvalue >> 4)+1) * 250, regvalue & 0x0f );

	regvalue = nrf24_get_register(NRF24_REG_RF_CH);
	printk(KERN_INFO "nrf24: RF channel: %i (%iMHz) \n", regvalue, regvalue + 2400);

	regvalue = nrf24_get_register(NRF24_REG_RF_SETUP);
	if (regvalue & 0x10)
		printk(KERN_INFO "nrf24: Force PLL lock \n");
	else
		printk(KERN_INFO "nrf24: Don't force PLL lock \n");

	if (regvalue & 0x8)
		printk(KERN_INFO "nrf24: Air data rate 2Mbps \n");
	else
		printk(KERN_INFO "nrf24: Air data rate 1Mbps \n");

	printk(KERN_INFO "nrf24: Output power in TX mode: %idBm \n", 
		-6 * (3 - ((regvalue & 6) >> 1)) );
		
	if (regvalue & 0x1)
		printk(KERN_INFO "nrf24: Setup LNA gain \n");
	else
		printk(KERN_INFO "nrf24: Don't setup LNA gain \n");

	nrf24_get_address_register(NRF24_REG_RX_ADDR_P0, result);
	printk(KERN_INFO "nrf24: RX Pipe 0 address: %x:%x:%x:%x:%x \n",
		result[0],
		result[1],
		result[2],
		result[3],
		result[4]);

	nrf24_get_address_register(NRF24_REG_RX_ADDR_P1, result);
	printk(KERN_INFO "nrf24: RX Pipe 1 address: %x:%x:%x:%x:%x \n",
		result[0],
		result[1],
		result[2],
		result[3],
		result[4]);
	nrf24_get_address_register(NRF24_REG_RX_ADDR_P2, result);
	printk(KERN_INFO "nrf24: RX Pipe 2 address: %x:%x:%x:%x:%x \n",
		result[0],
		result[1],
		result[2],
		result[3],
		result[4]);
	nrf24_get_address_register(NRF24_REG_RX_ADDR_P3, result);
	printk(KERN_INFO "nrf24: RX Pipe 3 address: %x:%x:%x:%x:%x \n",
		result[0],
		result[1],
		result[2],
		result[3],
		result[4]);
	nrf24_get_address_register(NRF24_REG_RX_ADDR_P4, result);
	printk(KERN_INFO "nrf24: RX Pipe 4 address: %x:%x:%x:%x:%x \n",
		result[0],
		result[1],
		result[2],
		result[3],
		result[4]);
	nrf24_get_address_register(NRF24_REG_RX_ADDR_P5, result);
	printk(KERN_INFO "nrf24: RX Pipe 5 address: %x:%x:%x:%x:%x \n",
		result[0],
		result[1],
		result[2],
		result[3],
		result[4]);
	nrf24_get_address_register(NRF24_REG_TX_ADDR, result);
	printk(KERN_INFO "nrf24: TX address: %x:%x:%x:%x:%x \n",
		result[0],
		result[1],
		result[2],
		result[3],
		result[4]);

	regvalue = nrf24_get_register(NRF24_REG_DYNPD);
	printk(KERN_INFO "nrf24: Dynamic payload length %x \n", regvalue);
}

static void nrf24_init_device()
{
	printk(KERN_INFO "nrf24: Clearing interrupt flags... \n");
  nrf24_write_register(NRF24_REG_STATUS,0x70,0x70);
	
	printk(KERN_INFO "nrf24: Set power up... \n");
	nrf24_write_register(NRF24_REG_CONFIG,0x2,0x2);
	mdelay(2);
}

static int nrf24_check_device()
{
	u8 ret = 0;
	ret = nrf24_get_register(NRF24_REG_STATUS);
	return (ret != NRF24_REG_STATUS_DEFAULT);
}


static int nrf24_get_register(u8 reg)
{
	u8 ret = 0;
	gpio_set_value(NRF24_GPIO_CSN, 0);
	ndelay(NRF24_SPI_HALF_CLK);
	nrf24_send_byte(NRF24_CMD_R_REGISTER | reg);
	ret = nrf24_send_byte(NRF24_CMD_NOP);
	gpio_set_value(NRF24_GPIO_CSN, 1);
	return ret;

}

static void nrf24_get_address_register(u8 reg, u8* result)
{
	int i;
	gpio_set_value(NRF24_GPIO_CSN, 0);
	ndelay(NRF24_SPI_HALF_CLK);
	nrf24_send_byte(NRF24_CMD_R_REGISTER | reg);
	for(i=0;i<5;i++)
	{
		 *(result + i) = nrf24_send_byte(NRF24_CMD_NOP);
	}
	gpio_set_value(NRF24_GPIO_CSN, 1);
}


static void nrf24_write_register(u8 reg, u8 value, u8 mask)
{
	u8 ret;
	gpio_set_value(NRF24_GPIO_CSN, 0);
	ndelay(NRF24_SPI_HALF_CLK);
	nrf24_send_byte(NRF24_CMD_R_REGISTER & reg);	  	
	ret = nrf24_send_byte(NRF24_CMD_NOP);

	gpio_set_value(NRF24_GPIO_CSN, 1);
	ndelay(NRF24_SPI_HALF_CLK);
	gpio_set_value(NRF24_GPIO_CSN, 0);
	ndelay(NRF24_SPI_HALF_CLK);

	nrf24_send_byte(NRF24_CMD_W_REGISTER | reg);
	nrf24_send_byte((ret & ~mask) | value);
	gpio_set_value(NRF24_GPIO_CSN, 1);
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
 
