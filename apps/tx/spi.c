#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "spi.h"

int spi0_file = 0;
struct spi_ioc_transfer xfer[2];
#define SPI_TST_AH 0x00
#define SPI_TST_AL 0x01
#define SPI_TST_DH 0x5A
#define SPI_TST_DL 0xC3

//////////
// Init SPIdev
//////////
int spi_init(char filename[40])
{
        int file;
    __u8    mode, lsb, bits;
    __u32 speed=2500000;
 
		if ((file = open(filename, O_RDWR)) < 0)
//		if((file = open("/dev/spidev32766.0", O_RDWR)) < 0)
        {
	        printf("Failed to open the bus (%d).", file);
            /* ERROR HANDLING; you can check errno to see what went wrong */
	        return 0;
		}
 
        ///////////////
        // Verifications
        ///////////////
        //possible modes: mode |= SPI_LOOP; mode |= SPI_CPHA; mode |= SPI_CPOL; mode |= SPI_LSB_FIRST; mode |= SPI_CS_HIGH; mode |= SPI_3WIRE; mode |= SPI_NO_CS; mode |= SPI_READY;
        //multiple possibilities using |
        /*
            if (ioctl(file, SPI_IOC_WR_MODE, &mode)<0)   {
                perror("can't set spi mode");
                return;
                }
        */
 
            if (ioctl(file, SPI_IOC_RD_MODE, &mode) < 0)
                {
                perror("SPI rd_mode");
                return 0;
                }
            if (ioctl(file, SPI_IOC_RD_LSB_FIRST, &lsb) < 0)
                {
                perror("SPI rd_lsb_fist");
                return 0;
                }
        //sunxi supports only 8 bits
        /*
            if (ioctl(file, SPI_IOC_WR_BITS_PER_WORD, (__u8[1]){8})<0)   
                {
                perror("can't set bits per word");
                return;
                }
        */
            if (ioctl(file, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) 
                {
                perror("SPI bits_per_word");
                return 0;
                }
        /*
            if (ioctl(file, SPI_IOC_WR_MAX_SPEED_HZ, &speed)<0)  
                {
                perror("can't set max speed hz");
                return;
                }
        */
            if (ioctl(file, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) 
                {
                perror("SPI max_speed_hz");
                return 0;
                }
     
 
//	printf("SPI device %s: spi mode %d, %d bits %sper word, %d Hz max\n", filename, mode, bits, lsb ? "(lsb first) " : "", speed);
	printf("SPI device opened: spi mode %d, %d bits %sper word, %d Hz max\n", mode, bits, lsb ? "(lsb first) " : "", speed);
 
    //xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = 3; /* Length of  command to write*/
    xfer[0].cs_change = 0; /* Keep CS activated */
    xfer[0].delay_usecs = 0, //delay in us
    xfer[0].speed_hz = 2500000, //speed
    xfer[0].bits_per_word = 8, // bites per word 8
 
    //xfer[1].rx_buf = (unsigned long) buf2;
    xfer[1].len = 4; /* Length of Data to read */
    xfer[1].cs_change = 0; /* Keep CS activated */
    xfer[0].delay_usecs = 0;
    xfer[0].speed_hz = 2500000;
    xfer[0].bits_per_word = 8;
 
    return file;
}
 
 
 
//////////
// Read n bytes from the 2 bytes address
//////////
 
int spi_read(int file, int address, unsigned char * buffer, int nbytes)
{
	unsigned char   buf[3];
    int status;
	if (buffer == NULL) 
	{
		return RES_SPI_WRONG_PARAM;
	}
	memset(buffer, 0, nbytes);
    buf[0] = 0x01;
	buf[1] = (address >> 8) & 0xFF;
	buf[2] = address & 0xFF;
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = 3; /* Length of  command to write*/
	xfer[1].rx_buf = (unsigned long)buffer;
    xfer[1].len = nbytes; /* Length of Data to read */
    status = ioctl(file, SPI_IOC_MESSAGE(2), xfer);
    if (status < 0)
	{
        perror("SPI_IOC_MESSAGE");
	    return RES_SPI_FILE_ERROR;
	}
    //printf("env: %02x %02x %02x\n", buf[0], buf[1], buf[2]);
    //printf("ret: %02x %02x %02x %02x\n", buf2[0], buf2[1], buf2[2], buf2[3]);
 
    return RES_SPI_SUCCESS;
}
 
//////////
// Write n bytes into the 2 bytes address
//////////
int spi_write(int file, int address, unsigned char * buffer, int nbytes)
{
	unsigned char   buf[MAX_SPI_WRITE + 3];
    int status;
	if (buffer == NULL) 
	{
		return RES_SPI_WRONG_PARAM;
	}
    buf[0] = 0x00;
	buf[1] = (address >> 8) & 0xFF;
	buf[2] = address & 0xFF;
	memcpy(&buf[3], buffer, nbytes);
    xfer[0].tx_buf = (unsigned long)buf;
    xfer[0].len = nbytes+3; /* Length of  command to write*/
    status = ioctl(file, SPI_IOC_MESSAGE(1), xfer);
    if (status < 0)
	{
		perror("SPI_IOC_MESSAGE");
		return RES_SPI_FILE_ERROR;
	}
    //printf("env: %02x %02x %02x\n", buf[0], buf[1], buf[2]);
    //printf("ret: %02x %02x %02x %02x\n", buf2[0], buf2[1], buf2[2], buf2[3]);
 
    return RES_SPI_SUCCESS;
}
