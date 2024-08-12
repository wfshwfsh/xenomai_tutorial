#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <rtdm/driver.h>


// BCM GPIO IN 23 -> Pin 16
#define GPIO_IN  23
// BCM GPIO OUT 22 -> Pin 15
#define GPIO_OUT 22




int gpio_pin_open(struct rtdm_fd *fd, int oflags)
{
	printk("[%s]\n", __FUNCTION__);
	
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	unsigned int gpio = rtdm_fd_minor(fd);
	int ret = 0;
	struct rtdm_gpio_pin *pin;

	pin = container_of(dev, struct rtdm_gpio_pin, dev);
	ret = gpio_request(gpio, pin->name);

	if (ret) {
		printk(XENO_ERR "failed to request pin %d : %d\n", gpio, ret);
		return ret;
	} else {
		chan->requested = true;
	}

	return 0;
}


static void gpio_pin_close(struct rtdm_fd *fd)
{
	printk("[%s]\n", __FUNCTION__);
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	unsigned int gpio = rtdm_fd_minor(fd);
	struct rtdm_gpio_pin *pin;

	if (chan->requested) {
		pin = container_of(dev, struct rtdm_gpio_pin, dev);
		release_gpio_irq(gpio, pin, chan);
	}
}

static int gpio_pin_ioctl_nrt(struct rtdm_fd *fd,
			      unsigned int request, void *arg)
{
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	unsigned int gpio = rtdm_fd_minor(fd);
	int ret = 0, val, trigger;
	struct rtdm_gpio_pin *pin;
	
	pin = container_of(dev, struct rtdm_gpio_pin, dev);

	switch (request) {
	case GPIO_RTIOC_DIR_OUT:
		ret = rtdm_safe_copy_from_user(fd, &val, arg, sizeof(val));
		if (ret)
			return ret;
		ret = gpio_direction_output(gpio, val);
		if (ret == 0) {
			chan->has_direction = true;
			chan->is_output = true;
		}
		break;
	case GPIO_RTIOC_DIR_IN:
		ret = gpio_direction_input(gpio);
		if (ret == 0)
			chan->has_direction = true;
		break;
	
		case GPIO_RTIOC_IRQEN:
		if (chan->is_interrupt) {
			return -EBUSY;
		}
		ret = rtdm_safe_copy_from_user(fd, &trigger,
					       arg, sizeof(trigger));
		if (ret)
			return ret;
		ret = request_gpio_irq(gpio, pin, chan, trigger);
		break;
	case GPIO_RTIOC_IRQDIS:
		if (chan->is_interrupt) {
			release_gpio_irq(gpio, pin, chan);
			chan->requested = false;
			chan->is_interrupt = false;
		}
		break;
	case GPIO_RTIOC_REQS:
		ret = gpio_request(gpio, pin->name);
		if (ret)
			return ret;
		else
			chan->requested = true;
		break;
	case GPIO_RTIOC_RELS:
		gpio_free(gpio);
		chan->requested = false;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}


static ssize_t gpio_pin_read_rt(struct rtdm_fd *fd,
				void __user *buf, size_t len)
{
	printk("[%s]\n", __FUNCTION__);
	
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	struct rtdm_gpio_readout rdo;
	struct rtdm_gpio_pin *pin;
	int ret;

	if (!chan->has_direction)
		return -EAGAIN;

	if (chan->is_output)
		return -EINVAL;

	pin = container_of(dev, struct rtdm_gpio_pin, dev);

	if (chan->want_timestamp) {
		if (len < sizeof(rdo))
			return -EINVAL;

		if (!(fd->oflags & O_NONBLOCK)) {
			ret = rtdm_event_wait(&pin->event);
			if (ret)
				return ret;
			rdo.timestamp = pin->timestamp;
		} else if (pin->monotonic_timestamp) {
			rdo.timestamp = rtdm_clock_read_monotonic();
		} else {
			rdo.timestamp = rtdm_clock_read();
		}

		len = sizeof(rdo);
		rdo.value = gpiod_get_raw_value(pin->desc);
		ret = rtdm_safe_copy_to_user(fd, buf, &rdo, len);
	} else {
		if (len < sizeof(rdo.value))
			return -EINVAL;

		if (!(fd->oflags & O_NONBLOCK)) {
			ret = rtdm_event_wait(&pin->event);
			if (ret)
				return ret;
		}

		len = sizeof(rdo.value);
		rdo.value = gpiod_get_raw_value(pin->desc);
		ret = rtdm_safe_copy_to_user(fd, buf, &rdo.value, len);
	}
	
	return ret ?: len;
}

static ssize_t gpio_pin_write_rt(struct rtdm_fd *fd,
				 const void __user *buf, size_t len)
{
	printk("[%s]\n", __FUNCTION__);
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	struct rtdm_gpio_pin *pin;
	int value, ret;

	if (len < sizeof(value))
		return -EINVAL;

	if (!chan->has_direction)
		return -EAGAIN;

	if (!chan->is_output)
		return -EINVAL;

	ret = rtdm_safe_copy_from_user(fd, &value, buf, sizeof(value));
	if (ret)
		return ret;

	pin = container_of(dev, struct rtdm_gpio_pin, dev);
	gpiod_set_raw_value(pin->desc, value);

	return sizeof(value);
}


static struct rtdm_fd_ops my_rtdm_gpio{
	.open		=	gpio_pin_open,
	.close		=	gpio_pin_close,
	.ioctl_nrt	=	gpio_pin_ioctl_nrt,
	.read_rt	=	gpio_pin_read_rt,
	.write_rt	=	gpio_pin_write_rt,
	//.select		=	gpio_pin_select,
};

static int __init exemple_init (void)
{
    struct rtdm_gpio_chip *rgc;
    size_t asize;
    int ret;

    if (gc->ngpio == 0)
	return ERR_PTR(-EINVAL);


    // 1. allocate rgc
    

    // 2. device class create and fill
    rgc->devclass = class_create(gc->owner, gc->label);

    // 3. driver
    
    // init gpio fops read/write/ioctl_rt/ioctl_nrt
    
    // 4. sysclass settings
    rtdm_drv_set_sysclass(&rgc->driver, rgc->devclass);

    rgc->gc = gc;
    rtdm_lock_init(&rgc->lock);

    ret = create_pin_devices(rgc);
    if (ret)
        class_destroy(rgc->devclass);
	
    return ret;
}

static void __exit exemple_exit (void)
{
    gpio_free(GPIO_OUT);
    gpio_free(GPIO_IN);
}

module_init(exemple_init);
module_exit(exemple_exit);
MODULE_LICENSE("GPL");
