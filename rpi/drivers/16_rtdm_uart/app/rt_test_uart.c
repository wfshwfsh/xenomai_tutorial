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
#include <rtdm/serial.h>

#define WRITE_FILE    "/dev/rtdm/rtser1"
#define READ_FILE     "/dev/rtdm/rtser1"


static const struct rtser_config read_config = {
	.config_mask       = 0xFFFF,
	.baud_rate         = 115200,
	.parity            = RTSER_DEF_PARITY,
	.data_bits         = RTSER_DEF_BITS,
	.stop_bits         = RTSER_DEF_STOPB,
	.handshake         = RTSER_DEF_HAND,
	.fifo_depth        = RTSER_DEF_FIFO_DEPTH,
	.rx_timeout        = RTSER_DEF_TIMEOUT,
	.tx_timeout        = RTSER_DEF_TIMEOUT,
	.event_timeout     = 1000000, /* 1 s */
	.timestamp_history = RTSER_RX_TIMESTAMP_HISTORY,
	.event_mask        = RTSER_EVENT_RXPEND,
};

static const struct rtser_config write_config = {
	.config_mask       = 0xFFFF,//RTSER_SET_BAUD | RTSER_SET_TIMESTAMP_HISTORY,
	.baud_rate         = 115200,
	.parity            = RTSER_DEF_PARITY,
	.data_bits         = RTSER_DEF_BITS,
	.stop_bits         = RTSER_DEF_STOPB,
	.handshake         = RTSER_DEF_HAND,
	.fifo_depth        = RTSER_DEF_FIFO_DEPTH,
	.rx_timeout        = RTSER_DEF_TIMEOUT,
	.tx_timeout        = RTSER_DEF_TIMEOUT,
	//.timestamp_history = RTSER_DEF_TIMESTAMP_HISTORY,
    //.event_mask        = RTSER_EVENT_MODEMHI|RTSER_EVENT_MODEMLO,
	/* the rest implicitly remains default */
};

int uart_fd  = -1;
int ret;

#define YES 'y'

int main (int argc, char *argv[])
{
    char input;
    int i, ret, tx_val=0,rx_val=0;
    unsigned char txbuf[]={0x31,0x32,0x33,0x61,0x62,0x63}, rxbuf[6]={0};
    unsigned char *tx_ptr=txbuf;
    
	/* open rtser0 */
	uart_fd = open(WRITE_FILE, 0);
	if (uart_fd < 0) {
		printf("can't open %s, error: %s \n", WRITE_FILE, strerror(errno));
		return 0;
	}
    

	/* writing write-config */
	ret = ioctl(uart_fd, RTSER_RTIOC_SET_CONFIG, &read_config);
	if (ret) {
		printf("error while RTSER_RTIOC_SET_CONFIG: %s\n", strerror(errno));
		goto error;
	}
	printf("write-config written \n");

#if 1    
    //for(i=0;i<sizeof(txbuf);i++){
    //    ret = rt_dev_write(uart_fd, tx_ptr++, 1);
    ret = rt_dev_write(uart_fd, txbuf, sizeof(txbuf));
        if(ret < 0) {
            printf("failed to write data \n");
        }
    //}
#endif

#if 1
    do {
    //    input = getchar(); // 获取用户输入的字符
    //} while (input != YES);
    
    ret = rt_dev_read(uart_fd, rxbuf, sizeof(rxbuf));
    if(ret < 0) {
        printf("failed to read data: %s\n", strerror(errno));
    } else {
        printf("RX: \n");
        for(i=0;i<ret;i++){
            printf("0x%02x ", rxbuf[i]);
        }
        printf("\n\n");
    }
    } while (input != YES);
#endif

error:
    /* close the device */
    if(uart_fd > 0) {
        ret = close(uart_fd);
        if (ret < 0) {
            rt_printf("ERROR : can't open device (%s)\n", strerror(-ret));
            fflush(stdout);
            return ret;
        }
    }
    
    return 0;
}