/*
 * Copyright (C) 2015 Atmel Corporation
 *		      Wenyou.Yang <wenyou.yang@atmel.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <atmel_lcd.h>
#include <debug_uart.h>
#include <dm.h>
#include <i2c.h>
#include <version.h>
#include <video.h>
#ifdef CONFIG_DM_VIDEO
#include <video_console.h>
#endif
#include <asm/io.h>
#include <asm/arch/at91_common.h>
#include <asm/arch/atmel_pio4.h>
#include <asm/arch/atmel_mpddrc.h>
#include <asm/arch/atmel_sdhci.h>
#include <asm/arch/clk.h>
#include <asm/arch/gpio.h>
#include <asm/arch/sama5d2.h>

#include <../drivers/mux7xx/rtl8307h/rtk_api.h>
#include <../drivers/mux7xx/rtl8307h/rtk_api_ext.h>
#include <../drivers/mux7xx/rtl8307h/rtl8307h_types.h> 

#define	CONFIG_SYS_I2C_SPEED	100000

//#define	MEMORY_SDRAM_TEST	1

DECLARE_GLOBAL_DATA_PTR;

static void board_usb_hw_init(void)
{
	atmel_pio4_set_pio_output(AT91_PIO_PORTB, 10, 1);
}

#if defined(CONFIG_DM_VIDEO) && defined(CONFIG_ATMEL_HLCD)
static int video_show_board_logo_info(void)
{
	ulong dram_size;
	int i;
	u32 len = 0;
	char buf[255];
	char *corp = "2017 Microchip Technology Inc.\n";
	char temp[32];
	struct udevice *dev, *con;
	const char *s;
	vidinfo_t logo_info;
	int ret;

	get_microchip_logo_info(&logo_info);

	len += sprintf(&buf[len], "%s\n", U_BOOT_VERSION);
	memcpy(&buf[len], corp, strlen(corp));
	len += strlen(corp);
	len += sprintf(&buf[len], "%s CPU at %s MHz\n", get_cpu_name(),
			strmhz(temp, get_cpu_clk_rate()));

	dram_size = 0;
	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++)
		dram_size += gd->bd->bi_dram[i].size;

	len += sprintf(&buf[len], "%ld MB SDRAM\n", dram_size >> 20);

	ret = uclass_get_device(UCLASS_VIDEO, 0, &dev);
	if (ret)
		return ret;

	ret = video_bmp_display(dev, logo_info.logo_addr,
				logo_info.logo_x_offset,
				logo_info.logo_y_offset, false);
	if (ret)
		return ret;

	ret = uclass_get_device(UCLASS_VIDEO_CONSOLE, 0, &con);
	if (ret)
		return ret;

	vidconsole_position_cursor(con, 0, logo_info.logo_height);
	for (s = buf, i = 0; i < len; s++, i++)
		vidconsole_put_char(con, *s);

	return 0;
}
#endif

#define I2C_MUX_PCA_ADDR	(0xE0>>1)
int select_i2c_ch_pca(u8 ch)
{
struct udevice *bus, *dev;
u8 addr, data[8];
int err;
int ret;
struct i2c_msg msg[2];
u8 i2c_addr[3], i2c_data[4];

	uclass_get_device_by_seq(UCLASS_I2C, 0, &bus);
	dm_i2c_probe(bus, I2C_MUX_PCA_ADDR, 0, &dev);		// AT24MAC_ADDR

	/* Selecting proper channel via PCA*/
	data[0] = 0x04 | ch;
	addr = 0x04 | ch; // I2C_MUX_PCA_ADDR;
	
	err = dm_i2c_write(dev, addr, data, 0);
	//ret = i2c_write(I2C_MUX_PCA_ADDR, 0x0, 1, &ch, 1);
	if (ret) {
		printf("PCA: failed to select proper channel.\n");
		return ret;
	}
#if 0
	msg[0].addr  = 0x54;
	msg[0].flags = 0;
	msg[0].len   = 3;
	msg[0].buf   = i2c_addr;
	msg[1].addr  = 0x54;
	msg[1].flags = I2C_M_STOP | I2C_M_RD;
	msg[1].len   = 4;
	msg[1].buf   = i2c_data;
	dm_i2c_xfer(dev, msg, 2);
#endif
	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
//rtk_port_link_ability_t		P4_ability;
rtk_port_link_ability_t		P5_ability;
rtk_port_link_ability_t		P6_ability;

rtk_port_link_ability_t		heac0;
rtk_port_link_ability_t		heac1; 

#ifdef MEMORY_SDRAM_TEST
	unsigned long dram_start;
	unsigned long dram_size;
	unsigned long dram_end;
	unsigned long dram_point;
	int test_res;
#endif

#ifdef CONFIG_DM_VIDEO
	video_show_board_logo_info();
#endif

#ifdef MEMORY_SDRAM_TEST
	dram_start = CONFIG_SYS_SDRAM_BASE + 0x0000000;
	dram_size = CONFIG_SYS_SDRAM_SIZE;
	dram_end = CONFIG_SYS_SDRAM_BASE + 0x6000000;
	dram_point = 0;
	test_res = 0;
	for(dram_point = dram_start; dram_point < dram_end; dram_point = dram_point + sizeof(unsigned long)){
//		*(volatile unsigned long*)dram_point = dram_point^0x5AA55AA5;
		*(volatile unsigned long*)dram_point = dram_point;
	}
	
	dram_start = CONFIG_SYS_SDRAM_BASE + 0x8000000;
	dram_end = CONFIG_SYS_SDRAM_BASE + 0xF000000;//CONFIG_SYS_SDRAM_SIZE;
	dram_point = 0;
	test_res = 0;
	for(dram_point = dram_start; dram_point < dram_end; dram_point = dram_point + sizeof(unsigned long)){
//		*(volatile unsigned long*)dram_point = dram_point^0x5AA55AA5;
		*(volatile unsigned long*)dram_point = dram_point;
//		short_delay();
	}

	dram_start = CONFIG_SYS_SDRAM_BASE + 0x0000000;
	dram_size = CONFIG_SYS_SDRAM_SIZE;
	dram_end = CONFIG_SYS_SDRAM_BASE + 0x6000000;
	dram_point = 0;
	test_res = 0;
	for(dram_point = dram_start; dram_point < dram_end; dram_point = dram_point + sizeof(unsigned long)){
//		if(((*(volatile unsigned long*)dram_point)^0x5AA55AA5) != dram_point){
		if(((*(volatile unsigned long*)dram_point)) != dram_point){
//			printf("DDR Test Failed at addr 0x%08x data: 0x%08x vs 0x%08x\n", dram_point, *(volatile unsigned long*)dram_point, dram_start^0x5AA55AA5);
			printf("DDR Test Failed at addr 0x%08x data: 0x%08x vs 0x%08x\n", dram_point, *(volatile unsigned long*)dram_point, dram_point);
			printf("next: %08x %08x %08x %08x %08x %08x %08x %08x\n", 
				*(volatile unsigned long*)(dram_point+sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+2*sizeof(unsigned long)), 
				*(volatile unsigned long*)(dram_point+3*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+4*sizeof(unsigned long)),
				*(volatile unsigned long*)(dram_point+5*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+6*sizeof(unsigned long)),
				*(volatile unsigned long*)(dram_point+7*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+8*sizeof(unsigned long)));
			test_res = 1;
			break;
		}
	}
	if(test_res == 0){
		printf("DDR Test from 0x%08x to 0x%08x Passed.\n", dram_start, dram_end);
	}
	
	dram_start = CONFIG_SYS_SDRAM_BASE + 0x8000000;
	dram_end = CONFIG_SYS_SDRAM_BASE + 0xF000000;//CONFIG_SYS_SDRAM_SIZE;
	dram_point = 0;
	test_res = 0;
	for(dram_point = dram_start; dram_point < dram_end; dram_point = dram_point + sizeof(unsigned long)){
//		if(((*(volatile unsigned long*)dram_point)^0x5AA55AA5) != dram_point){
		if(((*(volatile unsigned long*)dram_point)) != dram_point){
//			printf("DDR Test Failed at addr 0x%08x data: 0x%08x vs 0x%08x\n", dram_point, *(volatile unsigned long*)dram_point, dram_start^0x5AA55AA5);
			printf("DDR Test Failed at addr 0x%08x data: 0x%08x vs 0x%08x\n", dram_point, *(volatile unsigned long*)dram_point, dram_point);
			printf("next: %08x %08x %08x %08x %08x %08x %08x %08x\n", 
				*(volatile unsigned long*)(dram_point+sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+2*sizeof(unsigned long)), 
				*(volatile unsigned long*)(dram_point+3*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+4*sizeof(unsigned long)),
				*(volatile unsigned long*)(dram_point+5*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+6*sizeof(unsigned long)),
				*(volatile unsigned long*)(dram_point+7*sizeof(unsigned long)), *(volatile unsigned long*)(dram_point+8*sizeof(unsigned long)));
			test_res = 1;
			break;
		}
	}
	if(test_res == 0){
		printf("DDR Test from 0x%08x to 0x%08x Passed.\n", dram_start, dram_end);
	}
#endif
	
#if 1
//i	select_i2c_ch_pca(3);
	select_i2c_ch_pca(2);

	RTL8307H_I2C_init(); 

	rtk_port_linkAbility_get(PN_PORT5, &P5_ability );
	rtk_port_linkAbility_get(PN_PORT6, &P6_ability );

	P5_ability.link   = PORT_LINKUP;
	rtk_port_linkAbility_set(PN_PORT5, &P5_ability );
	
	P6_ability.link   = PORT_LINKUP;
	//P6_ability.speed  = PORT_SPEED_10M;
	rtk_port_linkAbility_set(PN_PORT6, &P6_ability );
	
	rtk_hec_mode_set(PN_PORT0, HEC_MODE_ENABLE);
	rtk_hec_mode_set(PN_PORT1, HEC_MODE_ENABLE); 

	printf("p5 : %d %d %d %d %d %d\n", 
		P5_ability.speed, P5_ability.duplex, P5_ability.link, P5_ability.nway, P5_ability.txpause, P5_ability.rxpause);
	printf("p6 : %d %d %d %d %d %d\n", 
		P6_ability.speed, P6_ability.duplex, P6_ability.link, P6_ability.nway, P6_ability.txpause, P6_ability.rxpause);
#endif

#if 0
	/* set basic IP info*/
	uclass_get_device_by_seq(UCLASS_I2C, 0, &bus);
	dm_i2c_probe(bus, (0x68>>1), 0, &dev);		// FPGA MAC info

	data[0] = 0x00;
	data[1] = 0xA0;
	data[2] = 0xE5;
	data[3] = 0x00;
	data[4] = 0x00;
	data[5] = 0x10;
	addr = 0x04 | ch; // I2C_MUX_PCA_ADDR;
	
	err = dm_i2c_write(dev, addr, data, 0);
#endif

	return 0;
}
#endif

#ifdef CONFIG_DEBUG_UART_BOARD_INIT
static void board_uart1_hw_init(void)
{
	atmel_pio4_set_a_periph(AT91_PIO_PORTD, 2, 1);	/* URXD1 */
	atmel_pio4_set_a_periph(AT91_PIO_PORTD, 3, 0);	/* UTXD1 */

	at91_periph_clk_enable(ATMEL_ID_UART1);
}

void board_debug_uart_init(void)
{
	board_uart1_hw_init();
}
#endif

#ifdef CONFIG_BOARD_EARLY_INIT_F
int board_early_init_f(void)
{
#ifdef CONFIG_DEBUG_UART
	debug_uart_init();
#endif

	return 0;
}
#endif

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;

#ifdef CONFIG_CMD_USB
	board_usb_hw_init();
#endif

	return 0;
}

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)CONFIG_SYS_SDRAM_BASE,
				    CONFIG_SYS_SDRAM_SIZE);
	return 0;
}

#ifdef CONFIG_CMD_I2C
static int set_ethaddr_from_eeprom(void)
{
	const int ETH_ADDR_LEN = 6;
	unsigned char ethaddr[ETH_ADDR_LEN];
	const char *ETHADDR_NAME = "ethaddr";
	struct udevice *bus, *dev;

	if (getenv(ETHADDR_NAME))
		return 0;

	if (uclass_get_device_by_seq(UCLASS_I2C, 0, &bus)) {
		printf("Cannot find I2C bus 1\n");
		return -1;
	}

	if (dm_i2c_probe(bus, AT24MAC_ADDR, 0, &dev)) {
		printf("Failed to probe I2C chip\n");
		return -1;
	}

	if (dm_i2c_read(dev, AT24MAC_REG, ethaddr, ETH_ADDR_LEN)) {
		printf("Failed to read ethernet address from EEPROM\n");
		return -1;
	}

	if (!is_valid_ethaddr(ethaddr)) {
		printf("The ethernet address read from EEPROM is not valid!\n");
		return -1;
	}

	return eth_setenv_enetaddr(ETHADDR_NAME, ethaddr);
}
#else
static int set_ethaddr_from_eeprom(void)
{
	return 0;
}
#endif

#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
	set_ethaddr_from_eeprom();

	return 0;
}
#endif

/* SPL */
#ifdef CONFIG_SPL_BUILD
void spl_board_init(void)
{
}

static void ddrc_conf(struct atmel_mpddrc_config *ddrc)
{
	ddrc->md = (ATMEL_MPDDRC_MD_DBW_32_BITS | ATMEL_MPDDRC_MD_DDR3_SDRAM);

	ddrc->cr = (ATMEL_MPDDRC_CR_NC_COL_10 |
		    ATMEL_MPDDRC_CR_NR_ROW_14 |
		    ATMEL_MPDDRC_CR_CAS_DDR_CAS5 |
		    ATMEL_MPDDRC_CR_DIC_DS |
		    ATMEL_MPDDRC_CR_DIS_DLL |
		    ATMEL_MPDDRC_CR_NB_8BANKS |
		    ATMEL_MPDDRC_CR_DECOD_INTERLEAVED |
		    ATMEL_MPDDRC_CR_UNAL_SUPPORTED);

	ddrc->rtr = 0x511;

	ddrc->tpr0 = (6 << ATMEL_MPDDRC_TPR0_TRAS_OFFSET |
		      3 << ATMEL_MPDDRC_TPR0_TRCD_OFFSET |
		      4 << ATMEL_MPDDRC_TPR0_TWR_OFFSET |
		      9 << ATMEL_MPDDRC_TPR0_TRC_OFFSET |
		      3 << ATMEL_MPDDRC_TPR0_TRP_OFFSET |
		      4 << ATMEL_MPDDRC_TPR0_TRRD_OFFSET |
		      4 << ATMEL_MPDDRC_TPR0_TWTR_OFFSET |
		      4 << ATMEL_MPDDRC_TPR0_TMRD_OFFSET);

	ddrc->tpr1 = (27 << ATMEL_MPDDRC_TPR1_TRFC_OFFSET |
		      29 << ATMEL_MPDDRC_TPR1_TXSNR_OFFSET |
		      0 << ATMEL_MPDDRC_TPR1_TXSRD_OFFSET |
		      3 << ATMEL_MPDDRC_TPR1_TXP_OFFSET);

	ddrc->tpr2 = (0 << ATMEL_MPDDRC_TPR2_TXARD_OFFSET |
		      0 << ATMEL_MPDDRC_TPR2_TXARDS_OFFSET |
		      0 << ATMEL_MPDDRC_TPR2_TRPA_OFFSET |
		      4 << ATMEL_MPDDRC_TPR2_TRTP_OFFSET |
		      7 << ATMEL_MPDDRC_TPR2_TFAW_OFFSET);
}

void mem_init(void)
{
	struct at91_pmc *pmc = (struct at91_pmc *)ATMEL_BASE_PMC;
	struct atmel_mpddr *mpddrc = (struct atmel_mpddr *)ATMEL_BASE_MPDDRC;
	struct atmel_mpddrc_config ddrc_config;
	u32 reg;

	ddrc_conf(&ddrc_config);

	at91_periph_clk_enable(ATMEL_ID_MPDDRC);
	writel(AT91_PMC_DDR, &pmc->scer);

	reg = readl(&mpddrc->io_calibr);
	reg &= ~ATMEL_MPDDRC_IO_CALIBR_RDIV;
	reg |= ATMEL_MPDDRC_IO_CALIBR_DDR3_RZQ_55;
	reg &= ~ATMEL_MPDDRC_IO_CALIBR_TZQIO;
	reg |= ATMEL_MPDDRC_IO_CALIBR_TZQIO_(100);
	writel(reg, &mpddrc->io_calibr);

	writel(ATMEL_MPDDRC_RD_DATA_PATH_SHIFT_TWO_CYCLE,
	       &mpddrc->rd_data_path);

	ddr3_init(ATMEL_BASE_MPDDRC, ATMEL_BASE_DDRCS, &ddrc_config);

	writel(0x3, &mpddrc->cal_mr4);
	writel(64, &mpddrc->tim_cal);
}

void at91_pmc_init(void)
{
	struct at91_pmc *pmc = (struct at91_pmc *)ATMEL_BASE_PMC;
	u32 tmp;

	/*
	 * while coming from the ROM code, we run on PLLA @ 492 MHz / 164 MHz
	 * so we need to slow down and configure MCKR accordingly.
	 * This is why we have a special flavor of the switching function.
	 */
	tmp = AT91_PMC_MCKR_PLLADIV_2 | \
	      AT91_PMC_MCKR_MDIV_3 | \
	      AT91_PMC_MCKR_CSS_MAIN;
	at91_mck_init_down(tmp);

	tmp = AT91_PMC_PLLAR_29 |
	      AT91_PMC_PLLXR_PLLCOUNT(0x3f) |
	      AT91_PMC_PLLXR_MUL(82) |
	      AT91_PMC_PLLXR_DIV(1);
	at91_plla_init(tmp);

	writel(0x0 << 8, &pmc->pllicpr);

	tmp = AT91_PMC_MCKR_H32MXDIV |
	      AT91_PMC_MCKR_PLLADIV_2 |
	      AT91_PMC_MCKR_MDIV_3 |
	      AT91_PMC_MCKR_CSS_PLLA;
	at91_mck_init(tmp);
}
#endif