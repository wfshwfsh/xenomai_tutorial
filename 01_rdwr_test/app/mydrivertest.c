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

#define DEVICE_NAME		"my_rtdm"

#define RTTST_RTDM_DEFER_CLOSE_CONTEXT   1
#define RTTST_RTDM_MAGIC_PRIMARY        0xfefbfefb
#define RTTST_RTDM_MAGIC_SECONDARY      0xa5b9a5b9


void clr_buf(char *buf, int buf_sz)
{
	memset(buf, 0, buf_sz);
}

void set_buf(char *buf, const char *ptr)
{
	strcpy(buf, ptr);
}

int main(int argc, char *argv)
{
	char buf[1024];
	ssize_t size;
	int device;
	int ret;
	clr_buf(buf, sizeof(buf));

	/* open the device */
	device = rt_dev_open(DEVICE_NAME, 0);
	if (device < 0) {
		printf("ERROR : can't open device %s (%s)\n",
		       DEVICE_NAME, strerror(-device));
		fflush(stdout);
		exit(1);
	}
	
	set_buf(buf, "ABCD123");

	/* first write */
	size = rt_dev_write (device, (const void *)buf, strlen(buf) + 1);
	printf("Write from device %s\t: %d bytes\n", DEVICE_NAME, size);

	/* first read */
	clr_buf(buf, sizeof(buf));
	size = rt_dev_read (device, (void *)buf, 1024);
	printf("Read in device %s\t: %s\n", DEVICE_NAME, buf);

	/* second read */
	clr_buf(buf, sizeof(buf));
	size = rt_dev_read (device, (void *)buf, 1024);
	printf("Read in device again%s\t: %d bytes\n", DEVICE_NAME, size);

	/* close the device */
	ret = rt_dev_close(device);
	if (ret < 0) {
		printf("ERROR : can't close device %s (%s)\n",
		       DEVICE_NAME, strerror(-ret));
		fflush(stdout);
		exit(1);
	}

	return 0;
}
