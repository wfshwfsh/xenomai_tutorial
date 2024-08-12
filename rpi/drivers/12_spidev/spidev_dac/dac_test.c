/* 参考: tools\spi\spidev_fdx.c */

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <linux/types.h>
#include <linux/spi/spidev.h>

//#define SPIOLED_FEQ 	10000000
#define SPIDAC_FEQ 	20000000
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))  

static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = SPIDAC_FEQ;
static uint16_t delay;

/* dac_test /dev/spidevB.D <val> */

static void pabort(const char *s)  
{  
    perror(s);  
    abort();  
}  

static void spi_init(int fd)
{
	int ret;
	
    /* 
     * spi mode 
     */  
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);  
    if (ret == -1)  
        pabort("can't set spi mode");  
  
    ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);  
    if (ret == -1)  
        pabort("can't get spi mode");  
  
    /* 
     * bits per word 
     */  
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);  
    if (ret == -1)  
        pabort("can't set bits per word");  
  
    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);  
    if (ret == -1)  
        pabort("can't get bits per word");  
  
    /* 
     * max speed hz 
     */  
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);  
    if (ret == -1)  
        pabort("can't set max speed hz");  
  
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);  
    if (ret == -1)  
        pabort("can't get max speed hz");  
  
    printf("spi mode: %d\n", mode);  
    printf("bits per word: %d\n", bits);  
    printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);  
}

static void transfer(int fd, int val)  
{
	int status;
	uint8_t tx[2] = {0};
	uint8_t rx[ARRAY_SIZE(tx)] = {0, };
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = ARRAY_SIZE(tx),
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };
	
	val <<= 2;     /* bit0,bit1 = 0b00 */
	val &= 0xFFC;  /* 只保留10bit */

	tx[1] = val & 0xff;
	tx[0] = (val>>8) & 0xff;
	
	status = ioctl(fd, SPI_IOC_MESSAGE(1), tr);
	if (status < 0) {
		printf("SPI_IOC_MESSAGE error code=%d \n", status);
		exit(-1);
	}

	/* 打印 */
	val = (rx[0] << 8) | (rx[1]);
	val >>= 2;
	printf("Pre val = %d\n", val);
}

int main(int argc, char **argv)
{
	int fd;
	unsigned int val;
	
	if (argc != 3)
	{
		printf("Usage: %s /dev/spidevB.D <val>\n", argv[0]);
		return 0;
	}

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		printf("can not open %s\n", argv[1]);
		return 1;
	}
	
	val = strtoul(argv[2], NULL, 0);
	
	spi_init(fd);
	transfer(fd, val);
	close(fd);
	return 0;
}

