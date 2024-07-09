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
#include <rtdm/uapi/gpio.h>

#define TASK_PRIO 90 // 99 is Highest RT priority, 0 is Lowest
#define TASK_MODE 0 // No flags
#define TASK_STKSZ 0 // default Stack size

#define TASK_PERIOD (500*1000*1000) // 0.5= 50,000,000 ns



#define DEV_GPIO	"/dev/rtdm/pinctrl-bcm2711/gpio"

#define GPIO_LOW  0
#define GPIO_HIGH 1

static int dev, ret;

void periodic_task (void *arg) { 
    
    int cnt=0, err, to_drv, len, led;
	char buf[32]={0};
    long diff=0;
    RTIME now, previous;
    char *devName = (char *)arg;
    
    previous= rt_timer_read();
    printf("before XENO loop:\n");
    
    if(rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD)))
        printf("rt_task_set_periodic failed \n");

    /* open the device */
    dev = rt_dev_open(devName, 0);
    if (dev < 0) {
        rt_printf("ERROR : can't open device %s (%s)\n",
                    devName, strerror(-dev));
        fflush(stdout);
        exit(1);
    }
	
	to_drv = GPIO_HIGH;
	err = rt_dev_ioctl (dev, GPIO_RTIOC_DIR_OUT, &to_drv);
	if (dev < 0) {
        rt_printf("ERROR : can't ioctl device %s (%s)\n",
                    devName, strerror(-dev));
		fflush(stdout);
        exit(1);
    }

    for (;;) {
		
		//ioctl or read/write
		to_drv = !to_drv;
		rt_printf("to_drv: %d\n", to_drv);
		//err = rt_dev_ioctl (dev, GPIO_RTIOC_DIR_OUT, &to_drv);
		len = rt_dev_write(dev, &to_drv, sizeof(to_drv));
        if(len < 0){
			exit(1);
		}
		
        //task migrates to primary mode with xeno API call
        rt_task_wait_period(NULL); //deschedule until next period.
        now = rt_timer_read(); //cureent time
    
        //task migrates to secondary mode with syscall
        //so printf may have unexpected impact on the timing
        diff = now - previous;
        
        //if(diff > 1.2*TASK_PERIOD)
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
    char dev_rtdm[128]={0};
	snprintf(dev_rtdm, sizeof(dev_rtdm), "%s%d", DEV_GPIO, 22);
    
    mlockall(MCL_CURRENT|MCL_FUTURE);

    //e1 = rt_timer_set_mode(TM_ONESHOT); // Set oneshot timer
    e2 = rt_task_create(&tA, "periodicTask", TASK_STKSZ, TASK_PRIO, TASK_MODE);
    
    //set period
    //e3 = rt_task_set_periodic(&tA, TM_NOW, rt_timer_ns2ticks(TASK_PERIOD));
    e4 = rt_task_start(&tA, &periodic_task, dev_rtdm); 
    
    if (e1 | e2 | e3 | e4) {
        fprintf(stderr, "Error launching periodic task....\n");
        rt_task_delete(&tA);
        exit(1);
    }

    printf("Press any key to end....\n");
    getchar();
    rt_task_delete(&tA);
    
    /* close the device */
    ret = rt_dev_close(dev);
    if (ret < 0) {
        printf("ERROR : can't open device %s (%s)\n",
               dev_rtdm, strerror(-ret));
        fflush(stdout);
        return ret;
    }

    return 0;
}
