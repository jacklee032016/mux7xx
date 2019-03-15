
#include "common.h"
#include "hardware.h"
#include "board.h"
#include "twi.h"
#include "debug.h"

#include "mux7xx.h"

#include "si5351bRegs.h"


unsigned int muxHwTwiBus(void)
{
	unsigned int bus = 0;

#if defined(CONFIG_TWI0)
	bus = 0;
#elif defined(CONFIG_TWI1)
	bus = 1;
#elif defined(CONFIG_TWI2)
	bus = 2;
#elif defined(CONFIG_TWI3)
	bus = 3;
#endif
	return bus;
}

/* channel in multiplexer */
unsigned int muxHwTwiSetMuxChannel(unsigned char channo)
{
	unsigned char	buffer[2];
	unsigned int bus;

	bus = muxHwTwiBus();
	
	buffer[0] = 0x04 | channo;	// set mux for si5351b 
	if(twi_write(bus, EXT_I2C_DEV_PCA9544, 0, 0, buffer, 1) )
	{
		dbg_info("PCA9544 %x, buffer %x failed\n", EXT_I2C_DEV_PCA9544, buffer[0] );
	}
	else
	{
		dbg_info("PCA9544 %x, buffer %x OK\n", EXT_I2C_DEV_PCA9544, buffer[0] );
	}

	return bus;
}

int  si5351bHwInit(void)
{
	int i;
	unsigned char	buffer[2];
	unsigned char reg_addr;
	unsigned int bus;

	bus = muxHwTwiSetMuxChannel(MUX_BUS_SI5351B);

	// Turn off the clocks
	buffer[0] = 0xff;
	if(twi_write(bus, EXT_I2C_DEV_SI5351B, 3, 1, buffer, 1) )
	{
		dbg_info("SI5351B, 3.1 failed\n");
	}

	for (i=0; i<SI5351B_REVB_REG_CONFIG_NUM_REGS; i++)
	{
		buffer[0] = si5351b_revb_registers[i].value;
		reg_addr = si5351b_revb_registers[i].address;
		if(twi_write(bus, EXT_I2C_DEV_SI5351B, reg_addr, 1, buffer, 1) )
		{
			dbg_info("SI5351B, %x. failed\n", reg_addr);
		}
		
	}

//	buffer[0] = 0xAC;
//	twi_write(bus, EXT_I2C_DEV_SI5351B, 177, 1, buffer, 1);

  // Turn on the clocks 
	buffer[0] = 0x00;
	if(twi_write(bus, EXT_I2C_DEV_SI5351B, 3, 1, buffer, 1) )
	{
		dbg_info("SI5351B, 3.1 again failed\n");
	}
	
	return 0;
}

int  si5351bTxCfg(void)
{
	unsigned char	buffer[2];
	unsigned int bus;

	bus = muxHwTwiSetMuxChannel(MUX_BUS_SI5351B);

  // Disable the clock 6
	buffer[0] = 0x8C;
	twi_write(bus, EXT_I2C_DEV_SI5351B, 0x16, 1, buffer, 1);
	buffer[0] = 0x00;
	twi_write(bus, EXT_I2C_DEV_SI5351B, 0x5A, 1, buffer, 1);
	
	return 0;
}


short extSensorGetTemperatureCelsius(void)
{/* 11 bits: sign bit + 10 bit value */
	unsigned char regVal =0;
	short temperature = 0;
	unsigned int bus;
//	unsigned char	buffer[2];


	bus = muxHwTwiSetMuxChannel(MUX_BUS_LM9524);

	if(twi_read(bus, EXT_I2C_DEV_SENSOR, EXT_I2C_SENSOR_LOCAL_TEMP_MSB, 1, &regVal, 1) )
	{
		dbg_info("LM95245 Temp(MSB) read failed\n");
		return temperature;
	}
	else
	{
		dbg_info("LM95245 Temp(MSB): %d\n", regVal);
	}
	
	temperature = (regVal<<8);
	if(twi_read(bus, EXT_I2C_DEV_SENSOR, EXT_I2C_SENSOR_LOCAL_TEMP_LSB, 1, &regVal, 1) )
	{
		dbg_info("LM95245 Temp(LSB) read failed\n");
		return temperature;
	}
	else
	{
		dbg_info("LM95245 Temp(LSB): %d\n", regVal);
	}

	/* only 3 bits in LSB register */
	temperature |=  (regVal>>5); //*0.125;// / 256.0;
      
	dbg_info("LM95245 Temp(LSB): %d*0.125;\n", (regVal>>5) );
	
	return temperature;
}



unsigned char  mdio_init(void)
{
	return 1;
}


unsigned char  mdio_write_addr(unsigned char  prtad, unsigned char  devad, unsigned int  data)
{
	unsigned char	buf[6];
	unsigned int bus;

	bus = muxHwTwiBus();

	buf[0] = MDIO_OPS_ADDR;		// op code
	buf[1] = prtad;
	buf[2] = devad;
	buf[3] = data;
	buf[4] = (data>>8);
	buf[5] = 0x01;	// start cmd
		
	twi_write(bus, EXT_I2C_DEV_MDIO, MDIO_OPS, 1, buf, 6);

	return 1;
}

unsigned char  mdio_write_data(unsigned char  prtad, unsigned char  devad, unsigned int  data)
{
	unsigned char	buf[6];
	unsigned int bus;

	bus = muxHwTwiBus();

	buf[0] = MDIO_OPS_WRITE;		// op code
	buf[1] = prtad;
	buf[2] = devad;
	buf[3] = data;
	buf[4] = (data>>8);
	buf[5] = 0x01;	// start cmd

	twi_write(bus, EXT_I2C_DEV_MDIO, MDIO_OPS, 1, buf, 6);
	return 1;
}

unsigned char  mdio_read_data(unsigned char  prtad, unsigned char  devad, unsigned int  *data)
{
	unsigned char	buf[6];
	unsigned int bus;

	bus = muxHwTwiBus();

	buf[0] = MDIO_OPS_READ;		// op code
	buf[1] = prtad;
	buf[2] = devad;
	buf[3] = 0x55;
	buf[4] = 0x55;
	buf[5] = 0x01;	// start cmd

	twi_write(bus, EXT_I2C_DEV_MDIO, MDIO_OPS, 1, buf, 6);

	twi_read(bus, EXT_I2C_DEV_MDIO, FPGA_DATA, 1, buf, 2);  // read data register
	*data = (buf[1]<<8 | buf[0]);
	
	return 1;
}

unsigned char  mdio_read_data_incr(unsigned char  prtad, unsigned char  devad, unsigned int  *data)
{
	unsigned char	buf[6];
	unsigned int bus;

	bus = muxHwTwiBus();

	buf[0] = MDIO_OPS_READ_INC;		// op code
	buf[1] = prtad;
	buf[2] = devad;
	buf[3] = 0xDE;
	buf[4] = 0xAD;
	buf[5] = 0x01;	// start cmd

	twi_write(bus, EXT_I2C_DEV_MDIO, MDIO_OPS, 1, buf, 6);

	twi_read(bus, EXT_I2C_DEV_MDIO, FPGA_DATA, 1, buf, 2);  // read data register
	*data = (buf[1]<<8 | buf[0]);
	return 1;
}

unsigned char  mdio_read_data_buf(unsigned int  *data)
{
	unsigned char	buf[6];
	unsigned int bus;

	bus = muxHwTwiBus();

	twi_read(bus, EXT_I2C_DEV_MDIO, FPGA_DATA, 1, buf, 2);  // read data register
	*data = (buf[1]<<8 | buf[0]);
	
	return 1;
}



void  dac_param(int  dac)
{
	unsigned char	twi_dev;
//	unsigned char	twi_addr;
	unsigned char	data[16];
	unsigned int bus = 0;
	unsigned char	cmd;
	unsigned char	pd;
	unsigned int  udac;

	udac = 0x7ff + dac;
	bus = muxHwTwiSetMuxChannel(MUX_BUS_MCP4706);
	
	twi_dev = (0XC0>>1);
	cmd = 0;
	pd = 0;
	data[0] = ((cmd&0X03) << 6) | ((pd&0X03)<<4) | ((udac>>8)&0x0f);
	data[1] = udac;
	twi_write(bus, twi_dev, 0, 0, data, 2);
	
}


#if WITH_MEM_TEST
#define TEST_SYS_SDRAM_PRINT 1
void ddr_memory_test (void)
{
	unsigned long dram_start;
	unsigned long dram_size;
	unsigned long dram_end;
	unsigned long dram_point;
	int test_res;

	dram_start = TEST_SYS_SDRAM_BASE;
	dram_size = TEST_SYS_SDRAM_SIZE;
	dram_end = TEST_SYS_SDRAM_BASE + TEST_SYS_SDRAM_SIZE;
	
	dram_point = 0;
	dram_start = TEST_SYS_SDRAM_BASE + 0x0000000;
	dram_end = TEST_SYS_SDRAM_BASE + 0x6000000;
	for(dram_point = dram_start; dram_point < dram_end; dram_point = dram_point + sizeof(unsigned long)){
		*(volatile unsigned long*)dram_point = dram_point^TEST_SYS_SDRAM_PTRN;
	}
	
	dram_point = 0;
	dram_start = TEST_SYS_SDRAM_BASE + 0x8000000;
	dram_end = TEST_SYS_SDRAM_BASE + 0xF000000;
	for(dram_point = dram_start; dram_point < dram_end; dram_point = dram_point + sizeof(unsigned long)){
		*(volatile unsigned long*)dram_point = dram_point^TEST_SYS_SDRAM_PTRN;
	}
	
	dram_point = 0; 
	test_res = 0;
	dram_start = TEST_SYS_SDRAM_BASE + 0x0000000;
	dram_end = TEST_SYS_SDRAM_BASE + 0x6000000;
	for(dram_point = dram_start; dram_point < dram_end; dram_point = dram_point + sizeof(unsigned long)){
		if(((*(volatile unsigned long*)dram_point)^TEST_SYS_SDRAM_PTRN) != dram_point){
			dbg_info("DDR Memory Test Failed at addr %x data: %x vs %x\n", dram_point, *(volatile unsigned long*)dram_point, dram_start^TEST_SYS_SDRAM_PTRN);
			dbg_info("next: %x %x %x %x %x %x %x %x\n", 
				*(volatile unsigned long*)(dram_point+sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+2*sizeof(unsigned long)), 
				*(volatile unsigned long*)(dram_point+3*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+4*sizeof(unsigned long)),
				*(volatile unsigned long*)(dram_point+5*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+6*sizeof(unsigned long)),
				*(volatile unsigned long*)(dram_point+7*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+8*sizeof(unsigned long)));
			test_res = 1;
			break;
		}
	}
	if(test_res == 0){
		dbg_info("DDR Memory Test from %x to %x Passed.\n", dram_start, dram_end);
	}
	
	test_res = 0;
	dram_start = TEST_SYS_SDRAM_BASE + 0x8000000;
	dram_end = TEST_SYS_SDRAM_BASE + 0xF000000;
	for(dram_point = dram_start; dram_point < dram_end; dram_point = dram_point + sizeof(unsigned long)){
		if(((*(volatile unsigned long*)dram_point)^TEST_SYS_SDRAM_PTRN) != dram_point){
			dbg_info("DDR Memory Test Failed at addr %x data: %x vs %x\n", dram_point, *(volatile unsigned long*)dram_point, dram_start^TEST_SYS_SDRAM_PTRN);
			dbg_info("next: %x %x %x %x %x %x %x %x\n", 
				*(volatile unsigned long*)(dram_point+sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+2*sizeof(unsigned long)), 
				*(volatile unsigned long*)(dram_point+3*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+4*sizeof(unsigned long)),
				*(volatile unsigned long*)(dram_point+5*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+6*sizeof(unsigned long)),
				*(volatile unsigned long*)(dram_point+7*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+8*sizeof(unsigned long)));
			test_res = 1;
			break;
		}
	}
	if(test_res == 0){
		dbg_info("DDR Memory Test from %x to %x Passed.\n", dram_start, dram_end);
	}
}
#endif



// PLL Initialization
int  _rxInitDrpDLL(unsigned char mode)
{
	unsigned char	twi_dev;
	unsigned char	twi_addr;
	unsigned char	data[16];
	unsigned int bus = 0;
	//unsigned int chan = 0;

	unsigned char	drp_daddr;
	unsigned drp_data;

	unsigned char	cycle;

	switch (mode)
	{
		case 0: cycle = 2;  break;	// x2, 297 mhz
		case 1: cycle = 4;  break;	// x1, 148.5 mhz
		case 2: cycle = 8;  break;	// /2, 74.25 mhz
		default: cycle = 16;  break;	// /4, 37.125 mhz
	}

	twi_dev = (0x6A>>1);
	twi_addr = 0;
	drp_daddr = 0x08; 	// clk0, reg 1
	// see xapp888 for ext info on drp prog of pll module
	drp_data = cycle | (cycle<<6) | (1<<12) | (0 << 13);	// div by (cycle+cycle)

	data[0] = (drp_data & 0xff);
	data[1] = (drp_data >> 8);
	data[2] = drp_daddr;
	data[3] = 0x03;  // drp rd/wr cycle on this i2c write
	twi_write(bus, twi_dev, twi_addr, 1, data, 4);

	twi_addr = 4;
	twi_read(bus, twi_dev, twi_addr, 1, data, 3);

	dbg_info("DRP read %x %x %x\n", data[0], data[1], data[2] );

	return 0;
}

/*
 * Initialize devices for HDMI Output operation
 * Rx side only
=*/

int  muxRxInitVideoParam(void)
{
#if 0
	/* Jack comments. 03.13, 2019 */
	unsigned char	twi_dev;
	unsigned char	twi_addr;
	unsigned char	data[16];
	unsigned int bus = 0;
	//unsigned int chan = 0;

	unsigned int x, y;
	unsigned int sync_x, sync_y;

	unsigned int sx, sy;
#endif


#if 0
#if 1
	x = 3840;
	y = 2160;
	sync_x = 560;
	sync_y = 90; //48;

	Sil9136_set_video_info(29700, 30, (x+sync_x), (y+sync_y-1));

	_rxInitDrpDLL(0);	// X 2 

#else
	x = 1920;
	y = 1080;
	sync_x = 280;
	sync_y = 45;

	Sil9136_set_video_info(14850, 60, (x+sync_x), (y+sync_y-1));

	_rxInitDrpDLL(1);	// X 1

#endif
#endif


#if 0
	/* Jack comments. 03.13, 2019 */
	twi_dev = EXT_I2C_DEV_FPGA_SYSTEM;

	twi_addr = 6;
	// video active
	// Number of active pixels 
	data[0] = (x & 0xff);
	data[1] = (x >> 8);
	// Number of active rows 
	data[2] = (y & 0xff);
	data[3] = (y >> 8);
	// Setting the active horizontal and vertical video size
	twi_write(bus, twi_dev, twi_addr, 1, data, 4);


	twi_addr = 10;
	sx = x + sync_x -1;
	sy = y + sync_y -1;
	// video size total
	// Total number of cycles per row 
	data[0] = (sx & 0xff);
	data[1] = (sx >> 8);
	data[2] = 0;
	data[3] = 0;
	// Total number of rows per frame
	data[4] = (sy & 0xff);
	data[5] = (sy >> 8);
	data[6] = 0;
	data[7] = 0;
	twi_write(bus, twi_dev, twi_addr, 1, data, 8);
#endif

	return 0;
}


/*
 * Initialize devices for HDMI Input operation
 * Tx side only
 */
int muxTxInitHdmi(void)
{
	return 0;
}


/* FPGA control routines for different platform */
int fpgaWriteRegister(FPGA_CTRL *fpgaCtrl, int type, unsigned int regAddr, unsigned char addrSize, unsigned char *data, unsigned int bytes)
{
	unsigned char  _devAddr = fpgaDeviceAddress(fpgaCtrl, type);

	if(twi_write(fpgaCtrl->bus, _devAddr, regAddr, addrSize, data, bytes))
	{
		dbg_info("FPGA write type %d reg %x failed\n", type, regAddr);
		return -1;
	}

	return 0;
}


int fpgaReadRegister(FPGA_CTRL *fpgaCtrl, int type, unsigned int regAddr, unsigned char addrSize, unsigned char *data, unsigned int bytes)
{
	unsigned char  _devAddr = fpgaDeviceAddress(fpgaCtrl, type);

	if(twi_read(fpgaCtrl->bus, _devAddr, regAddr, addrSize, data, bytes))
	{
		dbg_info("FPGA read type %d reg %x failed\n", type, regAddr);
		return -1;
	}

	return 0;
}


