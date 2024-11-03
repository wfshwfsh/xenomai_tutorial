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

static int dev, ret;


int main (int argc, char *argv[])
{
    int ret, tx_val=0,rx_val=0;
    unsigned char txbuf[2]={0x3c,0x3d}, rxbuf[2]={0};
    unsigned char *ibuf, *obuf;

    //mlockall(MCL_CURRENT|MCL_FUTURE);

    if (argc != 3)
    {
        printf("Usage: %s /dev/spidevB.D <val>\n", argv[0]);
        return 0;
    }

#if 1
    if(0 > rt_dev_write(dev, txbuf, sizeof(txbuf)))
    {
        printf("failed to write data\n");
    }
#endif
#if 1
    if(rt_dev_read(dev, rxbuf, sizeof(rxbuf)) < 0)
    {
        printf("failed to read data\n");
    }
    else
    {

        printf("rxbuf = 0x%02x 0x%02x \n", rxbuf[0], rxbuf[1]);
    }
#endif

    /* close the device */
    ret = close(dev);
    if (ret < 0) {
        rt_printf("ERROR : can't open device (%s)\n",
               strerror(-ret));
        fflush(stdout);
        return ret;
    }

    return 0;
}
