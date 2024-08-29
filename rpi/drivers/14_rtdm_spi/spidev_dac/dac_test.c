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
#include <linux/spi/spidev.h>
#include <rtdm/spi.h>


#define TASK_PRIO 90 // 99 is Highest RT priority, 0 is Lowest
#define TASK_MODE 0 // No flags
#define TASK_STKSZ 0 // default Stack size

#define TASK_PERIOD (1000*1000*1000) // 0.5= 50,000,000 ns


#define SEQ_SHIFT 24
struct frame_header {
	unsigned int seq: SEQ_SHIFT,
		crc : 8;
} __attribute__((packed));

/* We send a 32bit header followed by 32 bytes of payload. */
#define TRANSFER_SIZE (4 + sizeof(struct frame_header))


static int dev;
struct rtdm_spi_iobufs iobufs;
static unsigned char *i_area, *o_area;

int rt_spidev_init(const char *devName)
{
    int speed_hz = 40000000;
    struct rtdm_spi_config config;
    int err;
    void *p;

    /* open the device */
    dev = open(devName, 0);
    if (dev <= 0) {
        rt_printf("ERROR : can't open device %s (%s)\n",
                    devName, strerror(-dev));
        fflush(stdout);
        exit(1);
    }
    
    rt_printf("open sucess %d \n", dev);
    
    iobufs.io_len = 4; /* tx_len=rx_len=2 */
    err = ioctl (dev, SPI_RTIOC_SET_IOBUFS, &iobufs);
    if(err){
    	rt_printf("ERROR : ioctl %d\n", err);
    	goto RET;
    }

#if 1
    p = mmap(0, iobufs.map_len, PROT_READ|PROT_WRITE, MAP_SHARED, dev, 0);
    if (p == MAP_FAILED){
	
	return -EINVAL;
    }else{
    
        rt_printf("p = %p\n", p);
        rt_printf("input_area[%u], output_area[%u], mapping length=%u\n",
		     iobufs.i_offset, iobufs.o_offset, iobufs.map_len);

        i_area = p + iobufs.i_offset;
        o_area = p + iobufs.o_offset;
        rt_printf("o_area=%p, i_area=%p\n", o_area,i_area);
    }
#endif
#if 1
    config.mode = SPI_MODE_0;
    config.bits_per_word = 8;
    config.speed_hz = speed_hz;

    err = ioctl(dev, SPI_RTIOC_SET_CONFIG, &config);
    if(err){
        rt_printf("ERROR : ioctl %d\n", err);
	goto RET;
    }

    err = ioctl(dev, SPI_RTIOC_GET_CONFIG, &config);
    if(err){
        rt_printf("ERROR : ioctl %d\n", err);
    }else{
        rt_printf("speed=%u hz, mode=%#x, bits=%u\n",
		config.speed_hz, config.mode, config.bits_per_word);
    }
#endif

RET:
    return err;
}

int main (int argc, char *argv[])
{
    int ret, tx_val=0,rx_val=0;
    unsigned char txbuf[2]={0}, rxbuf[2]={0};
    unsigned char *ibuf, *obuf;

    //mlockall(MCL_CURRENT|MCL_FUTURE);

    if (argc != 3)
    {
        printf("Usage: %s /dev/spidevB.D <val>\n", argv[0]);
        return 0;
    }

    tx_val = strtoul(argv[2], NULL, 0);

    rt_spidev_init(argv[1]);
   
    tx_val <<= 2;
    tx_val &= 0xffc;
    txbuf[1] = tx_val & 0xff;
    txbuf[0] = (tx_val>>8) & 0xff;

#if 0    
    if(0 > rt_dev_write(dev, txbuf, sizeof(txbuf)))
    {
        printf("failed to write data\n");
    }
#endif
#if 0
    if(rt_dev_read(dev, rxbuf, sizeof(rxbuf)) < 0)
    {
    	printf("failed to read data\n");
    }
    else
    {
	rx_val = (rxbuf[0] << 8) | (rxbuf[1] & 0xff);
	rx_val >>= 2;

    	printf("pre val = %d\n", rx_val);
    }
#endif
    
    obuf = o_area;
    ibuf = i_area;
    rt_printf("o_area=%p, i_area=%p\n", o_area,i_area);

    obuf[0] = txbuf[0];
    obuf[1] = txbuf[1];

    ret = rt_dev_ioctl(dev, SPI_RTIOC_TRANSFER);
    if (ret < 0) {
    	rt_printf("Error: failed to ioctl tx/rx data\n");
    }else{
	rxbuf[0] = ibuf[0];
	rxbuf[1] = ibuf[1];

	rx_val = (rxbuf[0] << 8) | (rxbuf[1] & 0xff);
        rx_val >>= 2;
	rt_printf("pre val = %d\n", rx_val);
    }

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
