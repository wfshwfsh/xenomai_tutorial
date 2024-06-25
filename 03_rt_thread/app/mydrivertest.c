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


void *test_basic(void *arg)
{
    int dev[2], ret[2], err;
    int from_drv;
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "basic-%d", getpid());
	err = pthread_setname_np(pthread_self(), task_name);
    if (err)
		printf("pthread_setname_np(basic)\n");
    
    while(1)
    {
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
        
        rt_printf("open sucess %d %d \n", dev[0], dev[1]);
        //getchar();
        
        //err = rt_dev_ioctl (dev[0], RTTST_RTIOC_RTDM_PING_PRIMARY, &from_drv);    
        //rt_printf("from_drv 0x%0x \n", from_drv);
        
        //getchar();
        err = rt_dev_ioctl (dev[0], RTTST_RTIOC_RTDM_PING_SECONDARY, &from_drv);
        //rt_printf("from_drv 0x%0x \n", from_drv);
        
        //getchar();
        //err = rt_dev_ioctl (dev[0], RTTST_RTIOC_RTDM_DEFER_CLOSE, RTTST_RTDM_DEFER_CLOSE_CONTEXT);
        
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
    }
}

static int thread_test()
{
    struct sched_param param;
	pthread_attr_t attr;
	pthread_t tid;
    void *p;
    int ret;
    
    pthread_attr_init(&attr);
	param.sched_priority = 0;
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
	pthread_attr_setschedparam(&attr, &param);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    
    ret = pthread_create(&tid, &attr, test_basic, NULL);
    ret = pthread_join(tid, &p);
    
    return (int)(long)p;
}

int main(int argc, char *argv)
{
    thread_test();

    //test_basic();

	return 0;
}
