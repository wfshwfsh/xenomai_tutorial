#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>

#define OLED_IOC_INIT 			123
#define OLED_IOC_SET_POS 		124

//为0 表示命令，为1表示数据
#define OLED_CMD 	0
#define OLED_DATA 	1

static struct class *spidev_class;
static struct spi_device *oled;
static int major;
static struct gpio_desc *dc_gpio;

static void oled_set_dc_pin(int val)
{
	gpiod_set_value(dc_gpio, val);
}

static void spi_write_datas(const unsigned char *buf, int len)
{
	spi_write(oled, buf, len);
}

//static void oled_write_datas(const unsigned char *buf, int len)
//{
//	oled_set_dc_pin(1);//拉高，表示写入数据
//	spi_write_datas(buf, len);
//}

static void oled_write_cmd_data(unsigned char uc_val,unsigned char uc_type)
{
	if(OLED_CMD == uc_type)
	{
		//pull low, when transfer cmd
		oled_set_dc_pin(0);
	}
	else
	{
		//pull high, when transfer data
		oled_set_dc_pin(1);
	}
	
	spi_write_datas(&uc_val, 1);//写入
}

static void dc_pin_init(void)
{
	//set direction: output
	gpiod_direction_output(dc_gpio, 1);
}

#if 0
//100ask flow
static void oled_init(void)
{
	//oled init sequence
	oled_write_cmd_data(0xae,OLED_CMD);//关闭显示

	oled_write_cmd_data(0x00,OLED_CMD);//设置 lower column address
	oled_write_cmd_data(0x10,OLED_CMD);//设置 higher column address

	oled_write_cmd_data(0x40,OLED_CMD);//设置 display start line

	oled_write_cmd_data(0xB0,OLED_CMD);//设置page address

	oled_write_cmd_data(0x81,OLED_CMD);// contract control
	oled_write_cmd_data(0x66,OLED_CMD);//128

	oled_write_cmd_data(0xa1,OLED_CMD);//设置 segment remap

	oled_write_cmd_data(0xa6,OLED_CMD);//normal /reverse

	oled_write_cmd_data(0xa8,OLED_CMD);//multiple ratio
	oled_write_cmd_data(0x3f,OLED_CMD);//duty = 1/64

	oled_write_cmd_data(0xc8,OLED_CMD);//com scan direction

	oled_write_cmd_data(0xd3,OLED_CMD);//set displat offset
	oled_write_cmd_data(0x00,OLED_CMD);//

	oled_write_cmd_data(0xd5,OLED_CMD);//set osc division
	oled_write_cmd_data(0x80,OLED_CMD);//

	oled_write_cmd_data(0xd9,OLED_CMD);//ser pre-charge period
	oled_write_cmd_data(0x1f,OLED_CMD);//

	oled_write_cmd_data(0xda,OLED_CMD);//set com pins
	oled_write_cmd_data(0x12,OLED_CMD);//

	oled_write_cmd_data(0xdb,OLED_CMD);//set vcomh
	oled_write_cmd_data(0x30,OLED_CMD);//

	oled_write_cmd_data(0x8d,OLED_CMD);//set charge pump disable 
	oled_write_cmd_data(0x14,OLED_CMD);//

	oled_write_cmd_data(0xaf,OLED_CMD);//set dispkay on
}
#else
//doc init seq flow picture
static void oled_init(void)
{
	//oled init sequence
	// set display off
	oled_write_cmd_data(0xAE,OLED_CMD);
	
	//init setting
	
	//
	//oled_write_cmd_data(0x00,OLED_CMD);
	//oled_write_cmd_data(0x10,OLED_CMD);
	
	
	//set sidplay clock divide ration/oscilator freq
	oled_write_cmd_data(0xD5,OLED_CMD);
	oled_write_cmd_data(0x80,OLED_CMD);
	
	//multiplex ratio
	oled_write_cmd_data(0xA8,OLED_CMD);
	oled_write_cmd_data(0x3F,OLED_CMD);
	
	//set display offset
	oled_write_cmd_data(0xD3,OLED_CMD);
	oled_write_cmd_data(0x00,OLED_CMD);
	
	//set display start line
	oled_write_cmd_data(0x40,OLED_CMD);
	
	//
	//oled_write_cmd_data(0xB0,OLED_CMD);
	
	
	//charge pump
	oled_write_cmd_data(0x8D,OLED_CMD);
	oled_write_cmd_data(0x10,OLED_CMD);
	
	//segment remap
	oled_write_cmd_data(0xA1,OLED_CMD);
	
	//set COM output scan direction
	oled_write_cmd_data(0xC8,OLED_CMD);
	
	//set com pins hw config
	oled_write_cmd_data(0xDA,OLED_CMD);
	oled_write_cmd_data(0x12,OLED_CMD);
	
	//set contrast control
	oled_write_cmd_data(0x81,OLED_CMD);
	oled_write_cmd_data(0x66,OLED_CMD);
	
	//pre-charge period
	oled_write_cmd_data(0xD9,OLED_CMD);
	oled_write_cmd_data(0x22,OLED_CMD);
	
	//VCOMH Deselect level
	oled_write_cmd_data(0xDB,OLED_CMD);
	oled_write_cmd_data(0x30,OLED_CMD);
	
	//set entire display on/off
	oled_write_cmd_data(0xA4,OLED_CMD);
	
	//set normal/inverse display
	oled_write_cmd_data(0xa6,OLED_CMD);
	
	// set display on
	oled_write_cmd_data(0xAF,OLED_CMD);
}
#endif

static void OLED_DIsp_Set_Pos(int x, int y)
{
 	oled_write_cmd_data(0xb0+y,OLED_CMD);
	oled_write_cmd_data((x&0x0f),OLED_CMD); 
	oled_write_cmd_data(((x&0xf0)>>4)|0x10,OLED_CMD);
}   	      	   			 


static long spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	printk("[%s] cmd=%d\n", __FUNCTION__, cmd);
	int x,y;
	
	/* 根据cmd操作硬件 */
	switch (cmd)
	{
		case OLED_IOC_INIT: /* init */
		{
			dc_pin_init();
			oled_init();
			break;
		}

		case OLED_IOC_SET_POS: /* set pos */
		{
			x = arg & 0xff;
			y = (arg >> 8) & 0xff;
			OLED_DIsp_Set_Pos(x,y);
			break;
		}
	}

	return 0;
}

static ssize_t spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	printk("[%s] \n", __FUNCTION__);
	char *kbuf;
	int err;
	
	kbuf = kmalloc(count, GFP_KERNEL);
	//copy_from_user
	err = copy_from_user(kbuf, buf, count);
	
	//spi_write_datas
	oled_set_dc_pin(1);
	spi_write_datas(kbuf, count);
	
	kfree(kbuf);
	return count;
}

static int spidev_open (struct inode *inode, struct file *filp)
{
	printk("[%s] \n", __FUNCTION__);
	return 0;
}

static int spidev_release (struct inode *inode, struct file *filp)
{
	printk("[%s] \n", __FUNCTION__);
	return 0;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	.write =	spidev_write,
	.unlocked_ioctl = spidev_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,
};

static int spidev_probe(struct spi_device *spi)
{
	struct device *dev;
	int status;
	
	major = register_chrdev(0, "100ask_oled", &spidev_fops);
	if (major < 0)
		return major;

	spidev_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(spidev_class)) {
		unregister_chrdev(major, "100ask_oled");
		return PTR_ERR(spidev_class);
	}
	
	dev = device_create(spidev_class, NULL, MKDEV(major, 0), NULL, "100ask_oled");
	status = PTR_ERR_OR_ZERO(dev);
	
	//record spi_device
	oled = spi;
	
	//get dc-gpio
	dc_gpio = gpiod_get(&spi->dev, "dc", 0);
	
	return status;
}

static int spidev_remove(struct spi_device *spi)
{
	gpiod_put(dc_gpio);
	
	device_destroy(spidev_class, MKDEV(major, 0));
	class_destroy(spidev_class);
	unregister_chrdev(major, "100ask_oled");
	return 0;
}


static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "100ask,oled" },
	{},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"100ask_oled",
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,
};

static int __init spidev_init(void)
{
	return spi_register_driver(&spidev_spi_driver);
}

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
}

module_init(spidev_init);
module_exit(spidev_exit);

MODULE_AUTHOR("Will.Chen");
MODULE_DESCRIPTION("100ask spi device driver");
MODULE_LICENSE("GPL");
