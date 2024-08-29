/*
 * xmmap_module.c - This Xenomai-3 rtdm kernel module creates shared memory
 * and implements a .mmap file operation which a user-space app (xmmap_user.c) 
 * can access
 *
 * Copyright (c) 2021 Konstantin Smola <k@smo.la>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <rtdm/driver.h>
#include <linux/version.h>

#include "xmmap.h"   /* common memory layout definition, etc */

#define DEVICE_NAME "xenomai_mmap_test"
#define MAX_DEVICES 1

/* prototypes */
void xmmap_exit(void);
int xmmap_init(void);
int Initialize(void);
void Cleanup(void);
ssize_t xmmap_module_read(struct rtdm_fd *fd, void *buf, size_t nbyte);
int xmmap_module_open(struct rtdm_fd *fd, int oflags);
void xmmap_module_close(struct rtdm_fd *fd);
ssize_t xmmap_module_write(struct rtdm_fd *fd, const void *user_space_buffer, size_t nbyte);
int xmmap_module_ioctl(struct rtdm_fd *fd, unsigned int request, void *arg);
int xmmap_module_mmap(struct rtdm_fd *fd, struct vm_area_struct *vma);

struct rtdm_device *dev;

shm_test_layout_t *xmmap_shared_area_p = NULL;	/* will point to Xenomai shared memory area */

void *module_user_area;		// userspace start address

MODULE_LICENSE("GPL");		// Include this, throws a message about tainting the kernel without it
MODULE_AUTHOR("Xenomai_user");

int shm_size;			// for module main shared memory segment

typedef struct custom_config {
	int flag1;
} custom_config_t;

struct xmmap_module_context {
	rtdm_lock_t lock;
	custom_config_t config;
};

static struct rtdm_driver module_tmpl = {
	.profile_info = RTDM_PROFILE_INFO(xmmap_fake, RTDM_CLASS_EXPERIMENTAL, 0, 999),
	.device_flags = RTDM_NAMED_DEVICE,
	.device_count = MAX_DEVICES,
	.context_size = sizeof(struct xmmap_module_context),
	.ops = {
		.open = xmmap_module_open,
		.close = xmmap_module_close,
		.read_nrt = xmmap_module_read,
		.write_rt = xmmap_module_write,	// if this was .write_nrt, userland calls to write() resolved to stub not xmmap_module_write()
		.ioctl_rt = xmmap_module_ioctl,
		.ioctl_nrt = xmmap_module_ioctl,
		.mmap = xmmap_module_mmap,
		},
};

ssize_t xmmap_module_read(struct rtdm_fd *fd, void *buf, size_t nbyte)
{
	rtdm_printk("[%s]\n", __FUNCTION__);
	return 0;
}

ssize_t xmmap_module_write(struct rtdm_fd * fd, const void *user_space_buffer, size_t nbyte)
{
	rtdm_printk("[%s]\n", __FUNCTION__);
	return 0;
}

int xmmap_module_open(struct rtdm_fd *fd, int oflags)	// new XENO3.1
{
	printk("open\n");
	rtdm_printk("[%s]\n", __FUNCTION__);
	return 0;
}

void xmmap_module_close(struct rtdm_fd *fd)
{
	rtdm_printk("[%s]\n", __FUNCTION__);
	return;
}

int xmmap_module_ioctl(struct rtdm_fd *fd, unsigned int request, void *arg)
{
	rtdm_printk("[%s]\n", __FUNCTION__);
	return 0;
}

/* The mmap() operation handler which shares a vmalloc'd kernelspace address with a userspace process */
int xmmap_module_mmap(struct rtdm_fd *fd, struct vm_area_struct *vma)
{
	int ret;

	rtdm_printk("xmmap module: xmmap_module_mmap() vma_start: %ld vma_end: %ld vma_size: %ld", vma->vm_start, vma->vm_end, (vma->vm_end - vma->vm_start));

	module_user_area = (void *)vma->vm_start;

	// do the address conversion from kernelspace virt mem to userspace.
	ret = rtdm_mmap_vmem(vma, xmmap_shared_area_p);
	// Could also do rtdm_mmap_kmem(vma, shm_test_p)
	if (ret == 0)		// success
	{
		rtdm_printk("xmmap module: mmap of xmmap shared area successful.\n");
	} else
		printk("xmmap module: Failed to rtdm_mmap_vmem()\n");

	return 0;
}

int __init xmmap_init(void)
{
	int err;
	char *name;
	int rem;

	rtdm_printk("xmmap module: Initializing xmmap module\n");
	rtdm_printk("xmmap module: raw shared struct size: %ld\n", sizeof(shm_test_layout_t));

	shm_size = (sizeof(shm_test_layout_t) / PAGE_SIZE);
	rem = (sizeof(shm_test_layout_t) % PAGE_SIZE);
	if (rem)
		shm_size += 1;
	if (shm_size < 2)	// xenomai has a minimum requirement of 2 pages for mappable heaps...
		shm_size = 2;	// ... and PAGE_SIZE is 4096
	shm_size *= PAGE_SIZE;	//This was necessary in native skin calls, unclear whether necassry in POSIX

	rtdm_printk("xmmap module: shared memory size will be: %d\n", shm_size);

	// allocate mem to share. (could also do kmalloc() here)
	xmmap_shared_area_p = (shm_test_layout_t *) vmalloc(shm_size + RTDM_MAX_DEVNAME_LEN);

	rtdm_printk("xmmap module: shared memory address: %p\n", xmmap_shared_area_p);
	if (xmmap_shared_area_p) {
		xmmap_shared_area_p->data[0] = 'x';	// Initialize some shared data
		xmmap_shared_area_p->data[1] = 'e';
		xmmap_shared_area_p->data[2] = 'n';
		xmmap_shared_area_p->data[3] = 'o';
		xmmap_shared_area_p->dval = 123.456;
	} else {
		rtdm_printk("Failed to allocate shared mem!\n");	// in Cobalt kernel can't get errno
		return -1;
	}

	dev = kmalloc(sizeof(struct rtdm_device) + RTDM_MAX_DEVNAME_LEN, GFP_KERNEL);
	if (!dev) {
		rtdm_printk("xmmap module: Problem allocating a device !");
		xmmap_exit();
	}

	dev->driver = &module_tmpl;
	dev->label = "xeno_mmap%d";	// will show up as /dev/rtdm/xeno_mmap0
	name = (char *)(dev + 1);
	ksformat(name, RTDM_MAX_DEVNAME_LEN, dev->label, 0);
	err = rtdm_dev_register(dev);
	if (err != 0) {
		rtdm_printk("xmmap module: Problem with device registration, returned: %d\n", err);
		return err;
	} else
		rtdm_printk("xmmap module: device0 registration OK, returned: %d\n", err);

	return 0;
}

void xmmap_exit(void)
{
	int err;

	rtdm_printk("xmmap module: exiting...\n");

	if (xmmap_shared_area_p) {
		rtdm_printk("xmmap module: unmapping module shared memory at %p of size: %d\n", xmmap_shared_area_p,
			    shm_size);
		err = rtdm_munmap(module_user_area, shm_size);	// undo the mmap, freeing user shmem 
		if (err != 0)
			rtdm_printk("xmmap module: warning, module shm munmap returned: %d\n", err);
		vfree(xmmap_shared_area_p);	// if using vmalloc
	}

	rtdm_printk("xmmap module: Unloading Xenomai xmmap-module device...\n");
	if (dev) {
		rtdm_dev_unregister(dev);
		kfree(dev);
	}

	rtdm_printk("xmmap module: Unloaded\n");
}

module_init(xmmap_init);
module_exit(xmmap_exit);
