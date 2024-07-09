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
#include <pthread.h>
#include <error.h>
#include <rtdm/rtdm.h>
#include <rtdm/uapi/gpio.h>

#define DEV_GPIO	"/dev/rtdm/pinctrl-bcm2711/gpio"

#define PRNT_DBG(...)   rt_printf("[%s %d] ", __func__, __LINE__); printf(__VA_ARGS__); printf("\n")

#define GPIO_LOW  0
#define GPIO_HIGH 1

void *test_basic(void *arg)
{
    int dev, ret, err, to_drv, len;
    int from_drv;
    char task_name[16];
    char *devName = (char *)arg;
    snprintf(task_name, sizeof(task_name), "basic-%d", getpid());
    err = pthread_setname_np(pthread_self(), task_name);
    if (err)
		error(1, err, "pthread_setname_np(basic)");
    
    PRNT_DBG("devName %s", devName);
	
	to_drv = GPIO_HIGH;
    while(1)
    {
		/* open the device */
		dev = rt_dev_open(devName, 0);
		if (dev < 0) {
			rt_printf("ERROR : can't open device %s (%s)\n",
						devName, strerror(-dev));
			fflush(stdout);
			exit(1);
		}
		
		err = rt_dev_ioctl (dev, GPIO_RTIOC_DIR_OUT, &to_drv);
		if (dev < 0) {
			rt_printf("ERROR : can't ioctl device %s (%s)\n",
						devName, strerror(-dev));
			fflush(stdout);
			exit(1);
		}
		
		to_drv = !to_drv;
        //err = rt_dev_ioctl (dev, GPIO_RTIOC_DIR_OUT, &to_drv);
		len = rt_dev_write(dev, &to_drv, sizeof(to_drv));
        if(len < 0){
			exit(1);
		}
		
        /* close the device */
        ret = rt_dev_close(dev);
        if (ret < 0) {
            rt_printf("ERROR : can't open device %s (%s)\n",
                       devName, strerror(-ret));
            fflush(stdout);
            exit(1);
        }
        
        sleep(1);
    }
}

static int thread_test(char *devName)
{
    struct sched_param param;
    pthread_attr_t attr;
    pthread_t tid;
    void *p;
    int ret;

    PRNT_DBG("devName %s", devName);

    pthread_attr_init(&attr);
    param.sched_priority = 0;
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    ret = pthread_create(&tid, &attr, test_basic, devName);
    ret = pthread_join(tid, &p);

    return (int)(long)p;
}

int main(int argc, char *argv)
{
    char dev_rtdm[128]={0};
	snprintf(dev_rtdm, sizeof(dev_rtdm), "%s%d", DEV_GPIO, 22);
    
    thread_test(dev_rtdm);

    return 0;
}
