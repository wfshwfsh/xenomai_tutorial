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
 * This is an example that shows how RTDM devices can be used
 * with a user space program.
 *
 * The device mydriver stores data that you write into.
 * When you read from this device, previously stored data is returned,
 * and the internal buffer is erased.
 *
 *
 * To test this application, you just need to:
 *
 * $ export LD_LIBRARY_PATH=<path of xenomai>/lib
 * $ insmod mydriver.ko
 * $ ./mydrivertest
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <rtdm/rtdm.h>

#define DEV_BASIC0	"rtdm0"
#define DEV_BASIC1	"rtdm1"


#define RTTST_RTDM_DEFER_CLOSE_CONTEXT   1
#define RTTST_RTDM_MAGIC_PRIMARY        0xfefbfefb
#define RTTST_RTDM_MAGIC_SECONDARY      0xa5b9a5b9

#define RTDM_CLASS_TESTING          6
#define RTIOC_TYPE_TESTING              RTDM_CLASS_TESTING
#define RTTST_RTIOC_RTDM_DEFER_CLOSE \
        _IOW(RTIOC_TYPE_TESTING, 0x40, __u32)

#define RTTST_RTIOC_RTDM_ACTOR_GET_CPU \
        _IOR(RTIOC_TYPE_TESTING, 0x41, __u32)

#define RTTST_RTIOC_RTDM_PING_PRIMARY \
        _IOR(RTIOC_TYPE_TESTING, 0x42, __u32)

#define RTTST_RTIOC_RTDM_PING_SECONDARY \
        _IOR(RTIOC_TYPE_TESTING, 0x43, __u32)


void test_basic()
{
    char c;
    int dev[2], ret[2], err;
    int from_drv;
    
    while(1){
    /* open the device */
    dev[0] = rt_dev_open(DEV_BASIC0, 0);
    dev[1] = rt_dev_open(DEV_BASIC1, 0);
    
    if (dev[0] < 0 || dev[1] < 0) {
		printf("ERROR : can't open device %s (%s) or device %s (%s)\n",
		       DEV_BASIC0, strerror(-dev[0]), 
               DEV_BASIC1, strerror(-dev[1]));
		fflush(stdout);
		exit(1);
	}
    
    printf("open sucess %d %d \n", dev[0], dev[1]);
    //getchar();

    err = rt_dev_ioctl (dev[0], RTTST_RTIOC_RTDM_PING_PRIMARY, &from_drv);    
    printf("from_drv 0x%0x \n", from_drv);
    
    //getchar();
    err = rt_dev_ioctl (dev[0], RTTST_RTIOC_RTDM_PING_SECONDARY, &from_drv);
    printf("from_drv 0x%0x \n", from_drv);
    
    //getchar();
    err = rt_dev_ioctl (dev[0], RTTST_RTIOC_RTDM_DEFER_CLOSE, RTTST_RTDM_DEFER_CLOSE_CONTEXT);
    
    //getchar();

    /* close the device */
    ret[0] = rt_dev_close(dev[0]);
    ret[1] = rt_dev_close(dev[1]);
    if (ret[0] < 0 || ret[1] < 0) {
	printf("ERROR : can't open device %s (%s) or device %s (%s)\n",
	       DEV_BASIC0, strerror(-ret[0]), 
               DEV_BASIC1, strerror(-ret[1]));
	fflush(stdout);
	exit(1);
    }
    
    sleep(1);
    };
}

int main(int argc, char *argv)
{
	char buf[1024];
	ssize_t size;
	int device;
	int ret;

#if 0
	//得到缓存区最大值
	int max_size;
        size = rt_dev_ioctl (device, 0x01,&max_size );
        printf("get max data size of device %s\t: %d bytes\n", DEVICE_NAME, max_size);
	
	int  valid_size;
        size = rt_dev_ioctl (device, 0x02,&valid_size );
        printf("get valid data size of device %s\t: %d bytes\n", DEVICE_NAME, valid_size);

	set_buf(buf, "ABCD123");

	/* first write */
	size = rt_dev_write (device, (const void *)buf, strlen(buf) + 1);
	printf("Write from device %s\t: %d bytes\n", DEVICE_NAME, size);
	
	clr_buf(buf, sizeof(buf));

	size = rt_dev_ioctl (device, 0x02,&valid_size );
        printf("get valid data size of device %s\t: %d bytes\n", DEVICE_NAME, valid_size);	

	/* first read */
	size = rt_dev_read (device, (void *)buf, 1024);
	printf("Read in device %s\t: %s\n", DEVICE_NAME, buf);

	/* second read */
	size = rt_dev_read (device, (void *)buf, 1024);
	printf("Read in device again%s\t: %d bytes\n", DEVICE_NAME, size);
#endif
	
    test_basic();
	return 0;
}
