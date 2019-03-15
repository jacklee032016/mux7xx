#define MAX_SPI_WRITE 64

#define RES_SPI_SUCCESS 0
#define RES_SPI_FILE_ERROR 1
#define RES_SPI_WRONG_PARAM 2

#define SPI_0_FILENAME "/dev/spidev32766.0"

#define SPI_TST_ADDR 0x0001
#define SPI_TST_D0 0x5A
#define SPI_TST_D1 0xC3

int spi_init(char filename[40]);
int spi_read(int file, int address, unsigned char * buffer, int nbytes);
int spi_write(int file, int address, unsigned char * buffer, int nbytes);
