
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

#define SPI_CDEV_NAME    "100ask_dac"

#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);

struct spidev_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* TX/RX buffers are NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;
	u8			*tx_buffer;
	u8			*rx_buffer;
	u32			speed_hz;
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static struct class *spidev_class;

#define pr_debug printk

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int val;
	int err;
    struct spidev_data	*spidev;
	struct spi_device	*spi;
    
	unsigned char tx_buf[2];	
	unsigned char rx_buf[2];	

	struct spi_message	msg;
	struct spi_transfer	xfer[1];
	int status;

	memset(&xfer[0], 0, sizeof(xfer));
	
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);
    
        mutex_lock(&spidev->buf_lock);
	/* copy_from_user */
	err = copy_from_user(&val, (const void __user *)arg, sizeof(int));

	printk("spidev_ioctl get val from user: %d\n", val);

	/* 发起SPI传输:     */

	/* 1. 把val修改为正确的格式 */
	val <<= 2;     /* bit0,bit1 = 0b00 */
	val &= 0xFFC;  /* 只保留10bit */

	tx_buf[1] = val & 0xff;
	tx_buf[0] = (val>>8) & 0xff;	

	/* 2. 发起SPI传输同时写\读 */
	/* 2.1 构造transfer
	 * 2.2 加入message
	 * 2.3 调用spi_sync
	 */
	xfer[0].tx_buf = tx_buf;
	xfer[0].rx_buf = rx_buf;
	xfer[0].len = 2;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	
	status = spi_sync(spi, &msg);

	/* 3. 修改读到的数据的格式 */
	val = (rx_buf[0] << 8) | (rx_buf[1]);
	val >>= 2;

	/* copy_to_user */
	err = copy_to_user((void __user *)arg, &val, sizeof(int));
	mutex_unlock(&spidev->buf_lock);
    
	return 0;
}

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	if (!spidev->tx_buffer) {
		spidev->tx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->tx_buffer) {
			status = -ENOMEM;
			goto err_find_dev;
		}
	}

	if (!spidev->rx_buffer) {
		spidev->rx_buffer = kmalloc(bufsiz, GFP_KERNEL);
		if (!spidev->rx_buffer) {
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

	spidev->users++;
	filp->private_data = spidev;
	stream_open(inode, filp);

	mutex_unlock(&device_list_lock);
	return 0;

err_alloc_rx_buf:
	kfree(spidev->tx_buffer);
	spidev->tx_buffer = NULL;
err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct spidev_data	*spidev;
	int			dofree;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;

	spin_lock_irq(&spidev->spi_lock);
	/* ... after we unbound from the underlying device? */
	dofree = (spidev->spi == NULL);
	spin_unlock_irq(&spidev->spi_lock);

	/* last close? */
	spidev->users--;
	if (!spidev->users) {

		kfree(spidev->tx_buffer);
		spidev->tx_buffer = NULL;

		kfree(spidev->rx_buffer);
		spidev->rx_buffer = NULL;

		if (dofree)
			kfree(spidev);
		else
			spidev->speed_hz = spidev->spi->max_speed_hz;
	}

	mutex_unlock(&device_list_lock);

	return 0;
}


static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */

	.unlocked_ioctl = spidev_ioctl,
    .open =		spidev_open,
	.release =	spidev_release,
};

static int spidev_probe(struct spi_device *spi)
{
	printk("[%s]\n", __FUNCTION__);
	struct spidev_data	*spidev;
	int			status;
	unsigned long		minor;
    
	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	/* Initialize the driver data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);

	INIT_LIST_HEAD(&spidev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(spidev_class, &spi->dev, spidev->devt,
				    spidev, "100ask_dac");
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&spidev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	spidev->speed_hz = spi->max_speed_hz;

	if (status == 0)
		spi_set_drvdata(spi, spidev);
	else
		kfree(spidev);
    
    return status;
}

static int spidev_remove(struct spi_device *spi)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spin_unlock_irq(&spidev->spi_lock);

	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "100ask,dac" },
	{},
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);


static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"spidac",
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

static int __init spidev_init(void)
{
	int status;
	printk("[%s]\n", __FUNCTION__);

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, SPI_CDEV_NAME, &spidev_fops);
	if (status < 0)
		return status;
	
	printk("[%s]\n", __FUNCTION__);
	spidev_class = class_create(THIS_MODULE, "spi_dac");
	if (IS_ERR(spidev_class)) {
		unregister_chrdev(SPIDEV_MAJOR, SPI_CDEV_NAME);
		return PTR_ERR(spidev_class);
	}

	printk("[%s]\n", __FUNCTION__);
	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		class_destroy(spidev_class);
		unregister_chrdev(SPIDEV_MAJOR, SPI_CDEV_NAME);
	}

	printk("[%s]\n", __FUNCTION__);
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	unregister_chrdev(SPIDEV_MAJOR, SPI_CDEV_NAME);
}
module_exit(spidev_exit);

MODULE_AUTHOR("Andrea Paterniani, <a.paterniani@swapp-eng.it>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");
