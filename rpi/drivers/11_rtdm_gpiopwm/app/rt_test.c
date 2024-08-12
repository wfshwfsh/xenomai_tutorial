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
#include <rtdm/uapi/gpiopwm.h>

#define DEV_GPIOPWM "/dev/rtdm/gpiopwm"

#define GPIO_PWM_SERVO_CONFIG			\
{						\
	.duty_cycle	=	10,		\
	.range_min	=	950,		\
	.range_max	=	2050,		\
	.period		=	20000000,	\
	.gpio		=	22,		\
}

static struct gpiopwm config = GPIO_PWM_SERVO_CONFIG;

static int dev, ret;

int main (int argc, char *argv[])
{
    int ret, err;
    char devName[128]={0};

    if(argc < 2){
        printf("xxx.app <gpiopwm_id>\n");
	return 0;
    }else{
        printf("gpiopwm id= %s\n", argv[1]);
    }
    snprintf(devName, sizeof(devName), "%s%s", DEV_GPIOPWM, argv[1]);


    /* open the device */
    dev = rt_dev_open(devName, 0);
    if (dev < 0) {
        rt_printf("ERROR : can't open device %s (%s)\n",
                    devName, strerror(-dev));
        fflush(stdout);
        exit(1);
    }
    
	printf("press to set config\n");
	getchar();
    err = rt_dev_ioctl (dev, GPIOPWM_RTIOC_SET_CONFIG, &config);
	
	printf("press to start pwm\n");
	getchar();
	err = rt_dev_ioctl (dev, GPIOPWM_RTIOC_START, NULL);
	
	printf("press to stop pwm\n");
	getchar();
	err = rt_dev_ioctl (dev, GPIOPWM_RTIOC_STOP, NULL);
	
	printf("press to close pwm\n");
	getchar();
    /* close the device */
    ret = rt_dev_close(dev);
    if (ret < 0) {
        printf("ERROR : can't open device %s (%s)\n",
               devName, strerror(-ret));
        fflush(stdout);
        return ret;
    }

    return 0;
}
