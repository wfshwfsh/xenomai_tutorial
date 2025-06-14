#include <stdio.h>
//#include <sys/io.h>
#include <stdlib.h>  
#include <string.h> 
#include <sys/types.h>
#include <sys/mman.h>
#include <dirent.h>
#include <ctype.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>

#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <trank/native/timer.h>
#include <trank/native/task.h>
#include <rtdm/rtdm.h>


#define TASK_PRIO 90 // 99 is Highest RT priority, 0 is Lowest
#define TASK_MODE 0 // No flags
#define TASK_STKSZ 0 // default Stack size

#define TASK_PERIOD (10*1000*1000) // 0.5= 50,000,000 ns



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



static int dev[2], ret[2];

void periodic_task (void *arg) {
    
    int cnt=0, err, from_drv;
    long diff=0;
    RTIME now, previous;
    previous= rt_timer_read();
    printf("before XENO loop:\n");
    
    if(rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD)))
        printf("rt_task_set_periodic failed \n");
    
    //printf("Press Any key to start rt task\n");
    //getchar();

    /* open the device */
    dev[0] = rt_dev_open(DEV_BASIC0, 0);
    dev[1] = rt_dev_open(DEV_BASIC1, 0);
    
    if (dev[0] < 0 || dev[1] < 0) {
        printf("ERROR : can't open device %s (%s) or device %s (%s)\n",
               DEV_BASIC0, strerror(-dev[0]), 
               DEV_BASIC1, strerror(-dev[1]));
        fflush(stdout);
        return;
    }
    
    rt_printf("open sucess %d %d \n", dev[0], dev[1]);

    for (;;) {
        
        err = rt_dev_ioctl (dev[0], RTTST_RTIOC_RTDM_PING_PRIMARY, &from_drv);
        //rt_printf("from_drv 0x%0x \n", from_drv);
        //err = rt_dev_ioctl (dev[0], RTTST_RTIOC_RTDM_PING_SECONDARY, &from_drv);
        rt_printf("from_drv 0x%0x \n", from_drv);
        //err = rt_dev_ioctl (dev[0], RTTST_RTIOC_RTDM_DEFER_CLOSE, RTTST_RTDM_DEFER_CLOSE_CONTEXT);
        
        //task migrates to primary mode with xeno API call
        rt_task_wait_period(NULL); //deschedule until next period.
	//usleep(TASK_PERIOD/1000);
        now = rt_timer_read(); //cureent time
    
        //task migrates to secondary mode with syscall
        //so printf may have unexpected impact on the timing
        diff = now - previous;
        
        if(diff > 1.5*TASK_PERIOD)
	        rt_printf("Time elapsed: %ld.%04ld ms\n",
                    (long)diff / 1000000,
                    (long)diff % 1000000);
        
        previous = now;
    }
}

int main (int argc, char *argv[])
{
    int e1=0, e2, e3=0, e4;
    RT_TIMER_INFO info;
    RT_TASK tA;
    
    mlockall(MCL_CURRENT|MCL_FUTURE);
    

    //e1 = rt_timer_set_mode(TM_ONESHOT); // Set oneshot timer
    e2 = rt_task_create(&tA, "periodicTask", TASK_STKSZ, TASK_PRIO, TASK_MODE);
    
    //set period
    //e3 = rt_task_set_periodic(&tA, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
    e4 = rt_task_start(&tA, &periodic_task, NULL); 
    
    if (e1 | e2 | e3 | e4) {
        fprintf(stderr, "Error launching periodic task....\n");
        rt_task_delete(&tA);
        exit(1);
    }

    printf("Press any key to end....\n");
    getchar();
    rt_task_delete(&tA);
    
    /* close the device */
    ret[0] = rt_dev_close(dev[0]);
    ret[1] = rt_dev_close(dev[1]);
    if (ret[0] < 0 || ret[1] < 0) {
        printf("ERROR : can't open device %s (%s) or device %s (%s)\n",
               DEV_BASIC0, strerror(-ret[0]), 
               DEV_BASIC1, strerror(-ret[1]));
        fflush(stdout);
        return;
    }

    return 0;
}
