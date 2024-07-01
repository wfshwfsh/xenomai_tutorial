/***************************************************************************
 *   Copyright (C) 2007 by trem (Philippe Reynes)                          *
 *   tremyfr@yahoo.fr                                                      *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/**
 * This kernel driver demonstrates how an RTDM device can be set up.
 *
 * It is a simple device, only 4 operation are provided:
 *  - open:  start device usage
 *  - close: ends device usage
 *  - write: store transfered data in an internal buffer
 *  - read:  return previously stored data and erase buffer
 *  - ioctl：0x01 return max data size
 *           0x02  return valid data size
 */

#include <linux/module.h>
#include <rtdm/driver.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("will chen");

#define BUF_SIZE_MAX		1024
#define DEVICE_NAME		"mydriver"

#define RTDM_CLASS_TESTING      6
#define RTDM_SUBCLASS_RTDMTEST  3
#define RTTST_PROFILE_VER       2
#define RTTST_RTDM_DEFER_CLOSE_CONTEXT   1

#define RTIOC_TYPE_TESTING              RTDM_CLASS_TESTING

#define RTTST_RTDM_MAGIC_PRIMARY        0xfefbfefb
#define RTTST_RTDM_MAGIC_SECONDARY      0xa5b9a5b9

#define RTTST_RTIOC_RTDM_DEFER_CLOSE \
        _IOW(RTIOC_TYPE_TESTING, 0x40, __u32)

#define RTTST_RTIOC_RTDM_ACTOR_GET_CPU \
        _IOR(RTIOC_TYPE_TESTING, 0x41, __u32)

#define RTTST_RTIOC_RTDM_PING_PRIMARY \
        _IOR(RTIOC_TYPE_TESTING, 0x42, __u32)

#define RTTST_RTIOC_RTDM_PING_SECONDARY \
        _IOR(RTIOC_TYPE_TESTING, 0x43, __u32)



struct rtdm_basic_context {
	rtdm_timer_t close_timer;
	unsigned long close_counter;
	unsigned long close_deferral;
};

struct rtdm_actor_context {
	rtdm_task_t actor_task;
	unsigned int request;
	rtdm_event_t run;
	rtdm_event_t done;
	union {
		__u32 cpu;
	} args;
};

/**
 * The context of a device instance
 *
 * A context is created each time a device is opened and passed to
 * other device handlers when they are called.
 *
 */
typedef struct buffer_s {
	int size;
	char data[BUF_SIZE_MAX];
} buffer_t;

static buffer_t device_buf;

/**
 * Open the device
 *
 * This function is called when the device shall be opened.
 *
 */
static int simple_rtdm_open(struct rtdm_fd *fd, int oflag)
{
	printk(KERN_INFO "RT device opened\n");
	return 0;
}

/**
 * Close the device
 *
 * This function is called when the device shall be closed.
 *
 */
static void simple_rtdm_close(struct rtdm_fd *fd)
{
	printk(KERN_INFO "RT device closed\n");
}

/**
 * Read from the device
 *
 * This function is called when the device is read in non-realtime context.
 *
 */
static ssize_t simple_rtdm_read_rt(struct rtdm_fd *fd, void __user *buf, size_t size)
{
	return rtdm_safe_copy_to_user(fd, buf, device_buf.data, size);
}

/**
 * Write in the device
 *
 * This function is called when the device is written in non-realtime context.
 *
 */
static ssize_t simple_rtdm_write_rt(struct rtdm_fd *fd, const void __user *buf, size_t size)
{
	return rtdm_safe_copy_from_user(fd, device_buf.data, buf, size);
}

static int simple_rtdm_ioctl_rt(struct rtdm_fd *fd,
			    unsigned int request, void __user *arg)
{
	int ret, magic = RTTST_RTDM_MAGIC_PRIMARY;
	printk("ioctl_rt req:%d \n", request);

	switch (request) {
	case RTTST_RTIOC_RTDM_PING_PRIMARY:
		ret = rtdm_safe_copy_to_user(fd, arg, &magic,
					     sizeof(magic));
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

static int simple_rtdm_ioctl_nrt(struct rtdm_fd *fd,
			    unsigned int request, void __user *arg)
{
	struct rtdm_basic_context *ctx = rtdm_fd_to_private(fd);
	int ret = 0, magic = RTTST_RTDM_MAGIC_SECONDARY;
	printk("ioctl_nrt req:%d \n", request);

	switch (request) {
	case RTTST_RTIOC_RTDM_DEFER_CLOSE:
		ctx->close_deferral = (unsigned long)arg;
		if (ctx->close_deferral == RTTST_RTDM_DEFER_CLOSE_CONTEXT) {
			++ctx->close_counter;
			rtdm_fd_lock(fd);
			rtdm_timer_start(&ctx->close_timer, 300000000ULL, 0,
					RTDM_TIMERMODE_RELATIVE);
		}
		break;
	case RTTST_RTIOC_RTDM_PING_SECONDARY:
		ret = rtdm_safe_copy_to_user(fd, arg, &magic,
					     sizeof(magic));
		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}




static struct rtdm_driver rtdm_basic_driver = {
	.profile_info		= RTDM_PROFILE_INFO(rtdm_test_basic,
						    RTDM_CLASS_EXPERIMENTAL,
						    RTDM_SUBCLASS_RTDMTEST,
						    RTTST_PROFILE_VER),
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.device_count		= 2,
	.context_size		= sizeof(struct rtdm_basic_context),

	.ops = {
		.open      = simple_rtdm_open,
                .close     = simple_rtdm_close,
                .read_rt   = simple_rtdm_read_rt,
                .write_rt  = simple_rtdm_write_rt,
		.ioctl_rt  = simple_rtdm_ioctl_rt,
		.ioctl_nrt = simple_rtdm_ioctl_nrt,
	},
};

/**
 * This structure describe the simple RTDM device
 *
 */
static struct rtdm_device device = {
        .driver = &rtdm_basic_driver,
	.label  = "my_rtdm"
};

/**
 * This function is called when the module is loaded
 *
 * It simply registers the RTDM device.
 *
 */
static int __init simple_rtdm_init(void)
{
	printk("simple rtdm init\n");
	return rtdm_dev_register(&device);
}

/**
 * This function is called when the module is unloaded
 *
 * It unregister the RTDM device, polling at 1000 ms for pending users.
 *
 */
static void __exit simple_rtdm_exit(void)
{
	printk("simple rtdm exit\n");
	rtdm_dev_unregister(&device);
}

module_init(simple_rtdm_init);
module_exit(simple_rtdm_exit);
