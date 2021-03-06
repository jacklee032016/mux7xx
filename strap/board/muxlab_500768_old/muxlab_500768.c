/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2015, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "common.h"
#include "hardware.h"
#include "pmc.h"
#include "usart.h"
#include "debug.h"
#include "ddramc.h"
#include "gpio.h"
#include "timer.h"
#include "watchdog.h"
#include "string.h"

#include "arch/at91_pmc.h"
#include "arch/at91_rstc.h"
#include "arch/at91_pio.h"
#include "arch/at91_ddrsdrc.h"
#include "arch/at91_sfr.h"
#include "muxlab_500768.h"
#include "l2cc.h"
#include "act8865.h"
#include "twi.h"
#include "arch/tz_matrix.h"
#include "matrix.h"

#include "si5351b.h"
#include "mdio_drv.h"

#include <rtk_api.h>
#include <rtk_api_ext.h>
#include <rtl8307h_types.h> 

//#include <rx_lib.h> 


void fpag_init_tx(void);
void fpag_init_rx(void);

void  dac_param(int  dac);
void  init_hdmi(void);
void  init_video_param(void);
void  init_pll_drp(unsigned char mode);

char  init_4k[];


enum
{
    Y1Y0_RGB = 0,                   /* Color space in Y1Y0 of AV Infoframe */
    Y1Y0_YCBCR_422,
    Y1Y0_YCBCR_444,
    Y1Y0_INVALID
};


 unsigned char _acsamsung_edid[256UL + 1] = {
  0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x4C, 0x2D, 0x1F, 0x04, 0x35, 0x32, 0x59, 0x4D, 0x0B, 0x18, 0x01, 0x04, 0xC2, 0x33, 0x1D, 0x78, 0x2A, 0xEE, 0x91, 0xA3, 0x54, 0x4C, 0x99, 0x26, 0x0F, 0x50, 0x54, 0x2F, 0xCE, 0x00, 0x81, 0x80,
  0x81, 0x40, 0x81, 0x00, 0x95, 0x00, 0xB3, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x3B, 0x3D, 0x00, 0xA0, 0x80, 0x80, 0x21, 0x40, 0x30, 0x20, 0x35, 0x00, 0xFE, 0x1F, 0x11, 0x00, 0x00, 0x1A, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x38, 0x3C, 0x1E,
  0x51, 0x10, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x53, 0x79, 0x6E, 0x63, 0x4D, 0x61, 0x73, 0x74, 0x65, 0x72, 0x0A, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x48, 0x56, 0x4E, 0x51, 0x43, 0x30, 0x30,
  0x34, 0x33, 0x33, 0x0A, 0x20, 0x20, 0x00, 0xAD, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFD, 0x00
};

const unsigned char _acComplete_EDID_SEIKI_4K[256UL + 1] = {
  0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x4C, 0xAB, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x23, 0x17, 0x01, 0x03, 0x81, 0x56, 0x30, 0x78, 0x8A, 0xA5, 0x8E, 0xA6, 0x54, 0x4A, 0x9C, 0x26, 0x12, 0x45, 0x46, 0xAD, 0xCE, 0x00, 0x81, 0x40,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x74, 0x00, 0x30, 0xF2, 0x70, 0x5A, 0x80, 0xB0, 0x58, 0x8A, 0x00, 0x56, 0xE1, 0x31, 0x00, 0x00, 0x1E, 0x9A, 0x29, 0xA0, 0xD0, 0x51, 0x84, 0x22, 0x30,
  0x50, 0x98, 0x36, 0x00, 0x60, 0xE1, 0x31, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x00, 0xFD, 0x00, 0x32, 0x4B, 0x18, 0x3C, 0x0B, 0x00, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x53, 0x45, 0x33, 0x39, 0x55, 0x59, 0x30,
  0x34, 0x0A, 0x20, 0x20, 0x20, 0x20, 0x01, 0x65, 0x02, 0x03, 0x2A, 0x71, 0x4F, 0x06, 0x07, 0x02, 0x03, 0x15, 0x96, 0x11, 0x12, 0x13, 0x04, 0x14, 0x05, 0x1F, 0x90, 0x20, 0x23, 0x09, 0x07, 0x07, 0x83, 0x01, 0x00, 0x00, 0x6D, 0x03, 0x0C, 0x00,
  0x20, 0x00, 0x08, 0x3C, 0x20, 0x40, 0x68, 0x01, 0x02, 0x03, 0x8C, 0x0A, 0xD0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0C, 0x40, 0x55, 0x00, 0x56, 0xE1, 0x31, 0x00, 0x00, 0x18, 0x01, 0x1D, 0x80, 0x18, 0x71, 0x1C, 0x16, 0x20, 0x58, 0x2C, 0x25, 0x00,
  0x56, 0xE1, 0x31, 0x00, 0x00, 0x9E, 0x01, 0x1D, 0x80, 0xD0, 0x72, 0x1C, 0x16, 0x20, 0x10, 0x2C, 0x25, 0x80, 0x56, 0xE1, 0x31, 0x00, 0x00, 0x9E, 0x01, 0x1D, 0x00, 0xBC, 0x52, 0xD0, 0x1E, 0x20, 0xB8, 0x28, 0x55, 0x40, 0x56, 0xE1, 0x31, 0x00,
  0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x00
};


// 500768 only
/*
static void AQR105_hw_reset(void)
{
	pio_set_gpio_output(XAUI_RX_DC_RST, 0);
	pio_set_gpio_output(XAUI_RESET, 0);
	pio_set_value(XAUI_RESET, 1);
	udelay(500);
	pio_set_value(XAUI_RESET, 0);
}
*/
static void RTL8307H_hw_reset(void)
{
	/* reset realtek ethernet switch */
	pio_set_gpio_output(ETH_RST_N, 1);
	pio_set_value(ETH_RST_N, 0);
	udelay(500);
	pio_set_value(ETH_RST_N, 1);
}

// 500768 only
/*
static void HDMI_hw_reset(void)
{
	pio_set_gpio_output(EXPL_COMP_ENA, 1);

	// reset both HDMI chips
	pio_set_gpio_output(EXPL_RST_N, 1);
	pio_set_gpio_output(HDMI_RST_N, 1);
	pio_set_value(EXPL_RST_N, 0);
	pio_set_value(HDMI_RST_N, 0);
	udelay(5000);
	pio_set_value(EXPL_RST_N, 1);
	pio_set_value(HDMI_RST_N, 1);
}
*/
static void HDMI_hw_reset(void)
{
	/* reset HDMI chip */
	pio_set_gpio_output(HDMI_RST_N, 1);
	pio_set_value(HDMI_RST_N, 0);
	udelay(5000);
	pio_set_value(HDMI_RST_N, 1);
}
static void FPGA_hw_reset(void)
{
	/* reset FPGA chip */
	pio_set_gpio_output(FPGA_RST_N, 1);
	pio_set_value(FPGA_RST_N, 0);
	udelay(5000);
	pio_set_value(FPGA_RST_N, 1);
}

static void FPGA_reload(void)
{
	/* start load FPGA firmware */
	pio_set_gpio_output(FPGA_PROGRAM, 1);
	pio_set_value(FPGA_PROGRAM, 0);
	udelay(500);
	pio_set_value(FPGA_PROGRAM, 1);
}

static unsigned int wait_FPGA_done(void)
{
	unsigned int done_timeout = 800;
	/* wait until load FPGA firmware done */
	while(done_timeout > 0){
		udelay(1000);
		done_timeout--;
		if(pio_get_value(FPGA_DONE)){
			return 1;
		}
	}
	return 0;
}


static void LED_RESET(void)
{
	/* set LED pins to output */
	pio_set_gpio_output(LED_VIDEO, 1);
	pio_set_gpio_output(LED_ACT, 1);
	pio_set_gpio_output(LED_POWER, 1);
	pio_set_gpio_output(LED_LINK, 1);
}

unsigned read_dipsw(void)
{
const struct pio_desc pio_pins[] = {
	{"SW1", DIPSW_01, 1, PIO_DEFAULT, PIO_INPUT},
	{"SW2", DIPSW_02, 1, PIO_DEFAULT, PIO_INPUT},
	{"SW3", DIPSW_03, 1, PIO_DEFAULT, PIO_INPUT},
	{"SW4", DIPSW_04, 1, PIO_DEFAULT, PIO_INPUT},
	{"SEL1", RX_SELECT1, 1, PIO_DEFAULT, PIO_INPUT},
	{"SEL2", RX_SELECT2, 1, PIO_DEFAULT, PIO_INPUT},
	{"FPDN", FPGA_DONE, 1, PIO_DEFAULT, PIO_INPUT},
	{(char *)0, 0, 0, PIO_DEFAULT, PIO_INPUT},
	};

//	pio_set_gpio_input(RX_SELECT2, PIO_PULLUP);

	pio_configure(pio_pins);
	pmc_sam9x5_enable_periph_clk(AT91C_ID_PIOA);
	pmc_sam9x5_enable_periph_clk(AT91C_ID_PIOB);
	pmc_sam9x5_enable_periph_clk(AT91C_ID_PIOC);
	pmc_sam9x5_enable_periph_clk(AT91C_ID_PIOD);
	

	/* reset both HDMI chips */
	return( (pio_get_value(DIPSW_01) ?0x01:0) |
		(pio_get_value(DIPSW_02) ?0x02:0) |
		(pio_get_value(DIPSW_03) ?0x04:0) |
		(pio_get_value(DIPSW_04) ?0x08:0) |
		(pio_get_value(RX_SELECT1) ?0x10:0) |
		(pio_get_value(RX_SELECT2) ?0x20:0) );
}

// 500768 only
/*
static void pwm_init(void)
{
	const struct pio_desc pwm_pins[] = {
		{"PWMIR", CONFIG_SYS_PWM_IR, 0, PIO_DEFAULT, PIO_PERIPH_D},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pio_configure(pwm_pins);		
//	pmc_sam9x5_enable_periph_clk(CONFIG_SYS_DBGU_ID);
}
*/
static void at91_dbgu_hw_init(void)
{
	const struct pio_desc dbgu_pins[] = {
		{"RXD1", CONFIG_SYS_DBGU_RXD_PIN, 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"TXD1", CONFIG_SYS_DBGU_TXD_PIN, 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pio_configure(dbgu_pins);
	pmc_sam9x5_enable_periph_clk(CONFIG_SYS_DBGU_ID);
}

static void initialize_dbgu(void)
{
	unsigned int baudrate = 115200;

	at91_dbgu_hw_init();

	if (pmc_check_mck_h32mxdiv())
		usart_init(BAUDRATE(MASTER_CLOCK / 2, baudrate));
	else
		usart_init(BAUDRATE(MASTER_CLOCK, baudrate));
}

#if defined(CONFIG_MATRIX)
static int matrix_configure_slave(void)
{
	unsigned int ddr_port;
	unsigned int ssr_setting, sasplit_setting, srtop_setting;

	/*
	 * Matrix 0 (H64MX)
	 */

	/*
	 * 0: Bridge from H64MX to AXIMX
	 * (Internal ROM, Crypto Library, PKCC RAM): Always Secured
	 */

	/* 1: H64MX Peripheral Bridge */

	/* 2 ~ 9 DDR2 Port1 ~ 7: Non-Secure */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_128M);
	sasplit_setting = (MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_128M)
				| MATRIX_SASPLIT(1, MATRIX_SASPLIT_VALUE_128M)
				| MATRIX_SASPLIT(2, MATRIX_SASPLIT_VALUE_128M)
				| MATRIX_SASPLIT(3, MATRIX_SASPLIT_VALUE_128M));
	ssr_setting = (MATRIX_LANSECH_NS(0)
			| MATRIX_LANSECH_NS(1)
			| MATRIX_LANSECH_NS(2)
			| MATRIX_LANSECH_NS(3)
			| MATRIX_RDNSECH_NS(0)
			| MATRIX_RDNSECH_NS(1)
			| MATRIX_RDNSECH_NS(2)
			| MATRIX_RDNSECH_NS(3)
			| MATRIX_WRNSECH_NS(0)
			| MATRIX_WRNSECH_NS(1)
			| MATRIX_WRNSECH_NS(2)
			| MATRIX_WRNSECH_NS(3));
	/* DDR port 0 not used from NWd */
	for (ddr_port = 1; ddr_port < 8; ddr_port++) {
		matrix_configure_slave_security(AT91C_BASE_MATRIX64,
					(H64MX_SLAVE_DDR2_PORT_0 + ddr_port),
					srtop_setting,
					sasplit_setting,
					ssr_setting);
	}

	/*
	 * 10: Internal SRAM 128K
	 * TOP0 is set to 128K
	 * SPLIT0 is set to 64K
	 * LANSECH0 is set to 0, the low area of region 0 is the Securable one
	 * RDNSECH0 is set to 0, region 0 Securable area is secured for reads.
	 * WRNSECH0 is set to 0, region 0 Securable area is secured for writes
	 */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_128K);
	sasplit_setting = MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_64K);
	ssr_setting = (MATRIX_LANSECH_S(0)
			| MATRIX_RDNSECH_S(0)
			| MATRIX_WRNSECH_S(0));
	matrix_configure_slave_security(AT91C_BASE_MATRIX64,
					H64MX_SLAVE_INTERNAL_SRAM,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 11:  Internal SRAM 128K (Cache L2) */
	/* 12:  QSPI0 */
	/* 13:  QSPI1 */
	/* 14:  AESB */

	/*
	 * Matrix 1 (H32MX)
	 */

	/* 0: Bridge from H32MX to H64MX: Not Secured */

	/* 1: H32MX Peripheral Bridge 0: Not Secured */

	/* 2: H32MX Peripheral Bridge 1: Not Secured */

	/*
	 * 3: External Bus Interface
	 * EBI CS0 Memory(256M) ----> Slave Region 0, 1
	 * EBI CS1 Memory(256M) ----> Slave Region 2, 3
	 * EBI CS2 Memory(256M) ----> Slave Region 4, 5
	 * EBI CS3 Memory(128M) ----> Slave Region 6
	 * NFC Command Registers(128M) -->Slave Region 7
	 *
	 * NANDFlash(EBI CS3) --> Slave Region 6: Non-Secure
	 */
	srtop_setting =	MATRIX_SRTOP(6, MATRIX_SRTOP_VALUE_128M);
	srtop_setting |= MATRIX_SRTOP(7, MATRIX_SRTOP_VALUE_128M);
	sasplit_setting = MATRIX_SASPLIT(6, MATRIX_SASPLIT_VALUE_128M);
	sasplit_setting |= MATRIX_SASPLIT(7, MATRIX_SASPLIT_VALUE_128M);
	ssr_setting = (MATRIX_LANSECH_NS(6)
			| MATRIX_RDNSECH_NS(6)
			| MATRIX_WRNSECH_NS(6));
	ssr_setting |= (MATRIX_LANSECH_NS(7)
			| MATRIX_RDNSECH_NS(7)
			| MATRIX_WRNSECH_NS(7));
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_EXTERNAL_EBI,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 4: NFC SRAM (4K): Non-Secure */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_8K);
	sasplit_setting = MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_8K);
	ssr_setting = (MATRIX_LANSECH_NS(0)
			| MATRIX_RDNSECH_NS(0)
			| MATRIX_WRNSECH_NS(0));
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_NFC_SRAM,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 5:
	 * USB Device High Speed Dual Port RAM (DPR): 1M
	 * USB Host OHCI registers: 1M
	 * USB Host EHCI registers: 1M
	 */
	srtop_setting = (MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_1M)
			| MATRIX_SRTOP(1, MATRIX_SRTOP_VALUE_1M)
			| MATRIX_SRTOP(2, MATRIX_SRTOP_VALUE_1M));
	sasplit_setting = (MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_1M)
			| MATRIX_SASPLIT(1, MATRIX_SASPLIT_VALUE_1M)
			| MATRIX_SASPLIT(2, MATRIX_SASPLIT_VALUE_1M));
	ssr_setting = (MATRIX_LANSECH_NS(0)
			| MATRIX_LANSECH_NS(1)
			| MATRIX_LANSECH_NS(2)
			| MATRIX_RDNSECH_NS(0)
			| MATRIX_RDNSECH_NS(1)
			| MATRIX_RDNSECH_NS(2)
			| MATRIX_WRNSECH_NS(0)
			| MATRIX_WRNSECH_NS(1)
			| MATRIX_WRNSECH_NS(2));
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_USB,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	return 0;
}

static unsigned int security_ps_peri_id[] = {
	0,
};

static int matrix_config_periheral(void)
{
	unsigned int *peri_id = security_ps_peri_id;
	unsigned int array_size = sizeof(security_ps_peri_id) / sizeof(unsigned int);
	int ret;

	ret = matrix_configure_peri_security(peri_id, array_size);
	if (ret)
		return -1;

	return 0;
}

static int matrix_init(void)
{
	int ret;

	matrix_write_protect_disable(AT91C_BASE_MATRIX64);
	matrix_write_protect_disable(AT91C_BASE_MATRIX32);

	ret = matrix_configure_slave();
	if (ret)
		return -1;

	ret = matrix_config_periheral();
	if (ret)
		return -1;

	return 0;
}
#endif	/* #if defined(CONFIG_MATRIX) */

#if defined(CONFIG_DDR3)
static void ddramc_reg_config(struct ddramc_register *ddramc_config)
{
	ddramc_config->mdr = (AT91C_DDRC2_DBW_32_BITS
				| AT91C_DDRC2_MD_DDR3_SDRAM);

//	ddramc_config->mdr = (AT91C_DDRC2_DBW_16_BITS
//				| AT91C_DDRC2_MD_DDR3_SDRAM);

	ddramc_config->cr = (AT91C_DDRC2_NC_DDR10_SDR9
				| AT91C_DDRC2_NR_14
				| AT91C_DDRC2_CAS_5
				| AT91C_DDRC2_DISABLE_DLL
				| AT91C_DDRC2_WEAK_STRENGTH_RZQ7
				| AT91C_DDRC2_NB_BANKS_8
				| AT91C_DDRC2_DECOD_INTERLEAVED
				| AT91C_DDRC2_UNAL_SUPPORTED);

	/*
	 * According to MT41K128M16 datasheet
	 * Maximum fresh period: 64ms, refresh count: 8k
	 */
#ifdef CONFIG_BUS_SPEED_166MHZ
	/* Refresh Timer is (64ms / 8k) * 166MHz = 1297(0x511) */
	ddramc_config->rtr = 0x511;

	/*
	 * According to the sama5d2 datasheet and the following values:
	 * T Sens = 0.75%/C, V Sens = 0.2%/mV, T driftrate = 1C/sec and V driftrate = 15 mV/s
	 * Warning: note that the values T driftrate and V driftrate are dependent on
	 * the application environment.
	 * ZQCS period is 1.5 / ((0.75 x 1) + (0.2 x 15)) = 0.4s
	 * If tref is 7.8us, we have: 400000 / 7.8 = 51282(0xC852)
	 * */
	ddramc_config->cal_mr4r = AT91C_DDRC2_COUNT_CAL(0xC852);

	/* DDR3 ZQCS */
	ddramc_config->tim_calr = AT91C_DDRC2_ZQCS(64);

	/* Assume timings for 8ns min clock period */
	ddramc_config->t0pr = (AT91C_DDRC2_TRAS_(6)
			| AT91C_DDRC2_TRCD_(3)
			| AT91C_DDRC2_TWR_(4)
			| AT91C_DDRC2_TRC_(9)
			| AT91C_DDRC2_TRP_(3)
			| AT91C_DDRC2_TRRD_(4)
			| AT91C_DDRC2_TWTR_(4)
			| AT91C_DDRC2_TMRD_(4));

	ddramc_config->t1pr = (AT91C_DDRC2_TRFC_(27)
			| AT91C_DDRC2_TXSNR_(29)
			| AT91C_DDRC2_TXSRD_(0)
			| AT91C_DDRC2_TXP_(3));

	ddramc_config->t2pr = (AT91C_DDRC2_TXARD_(0)
			| AT91C_DDRC2_TXARDS_(0)
			| AT91C_DDRC2_TRPA_(0)
			| AT91C_DDRC2_TRTP_(4)
			| AT91C_DDRC2_TFAW_(7));
#else
#error "No CLK setting defined"
#endif
}

static void ddramc_init(void)
{
	struct ddramc_register ddramc_reg;
	unsigned int reg;

	ddramc_reg_config(&ddramc_reg);

	pmc_sam9x5_enable_periph_clk(AT91C_ID_MPDDRC);
	pmc_enable_system_clock(AT91C_PMC_DDR);

	/* MPDDRC I/O Calibration Register */
	reg = readl(AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR);
	reg &= ~AT91C_MPDDRC_RDIV;
	reg |= AT91C_MPDDRC_RDIV_DDR2_RZQ_50;
	reg &= ~AT91C_MPDDRC_TZQIO;
	reg |= AT91C_MPDDRC_TZQIO_(100);
	writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR));

	writel(AT91C_MPDDRC_RD_DATA_PATH_TWO_CYCLES,
			(AT91C_BASE_MPDDRC + MPDDRC_RD_DATA_PATH));

	ddr3_sdram_initialize(AT91C_BASE_MPDDRC, AT91C_BASE_DDRCS, &ddramc_reg);

	ddramc_dump_regs(AT91C_BASE_MPDDRC);
}

#elif defined(CONFIG_LPDDR1)
static void lpddr1_reg_config(struct ddramc_register *ddramc_config)
{
	ddramc_config->mdr = (AT91C_DDRC2_DBW_32_BITS |
			      AT91C_DDRC2_MD_LP_DDR_SDRAM);

	/* 14 Row bits, 10 Column bits */
	ddramc_config->cr = (AT91C_DDRC2_NC_DDR11_SDR10 |
			     AT91C_DDRC2_NR_14 |
			     AT91C_DDRC2_CAS_3 |
			     AT91C_DDRC2_NDQS_DISABLED |
			     AT91C_DDRC2_UNAL_SUPPORTED);

	ddramc_config->lpr = 0;

	/*
	 * According to MT46H128M16LF-5 IT datasheet
	 * Maximum fresh period: 64ms, refresh count: 8k
	 */
#ifdef CONFIG_BUS_SPEED_166MHZ
	/* Refresh Timer is (64ms / 8k) * 166MHz = 1297(0x511) */
	ddramc_config->rtr = 0x511;

	/* Assume the timings for 6ns min clock period */
	ddramc_config->t0pr = (AT91C_DDRC2_TRAS_(7) |
			       AT91C_DDRC2_TRCD_(3) |
			       AT91C_DDRC2_TWR_(3) |
			       AT91C_DDRC2_TRC_(10) |
			       AT91C_DDRC2_TRP_(3) |
			       AT91C_DDRC2_TRRD_(2) |
			       AT91C_DDRC2_TWTR_(2) |
			       AT91C_DDRC2_TMRD_(2));

	ddramc_config->t1pr = (AT91C_DDRC2_TRFC_(12)  |
			       AT91C_DDRC2_TXSNR_(19) |
			       AT91C_DDRC2_TXSRD_(0) |
			       AT91C_DDRC2_TXP_(2));

	ddramc_config->t2pr = (AT91C_DDRC2_TXARD_(0) |
			       AT91C_DDRC2_TXARDS_(0) |
			       AT91C_DDRC2_TRPA_(0) |
			       AT91C_DDRC2_TRTP_(0) |
			       AT91C_DDRC2_TFAW_(0));
#else
#error "No CLK setting defined"
#endif
}

static void lpddr1_init(void)
{
	struct ddramc_register ddramc_reg;
	unsigned int reg;

	lpddr1_reg_config(&ddramc_reg);

	pmc_sam9x5_enable_periph_clk(AT91C_ID_MPDDRC);
	pmc_enable_system_clock(AT91C_PMC_DDR);

	/*
	 * Before starting the initialization sequence, the user must force
	 * the DDR_DQ and DDR_DQS input buffers to always on by setting
	 * the FDQIEN and FDQSIEN bits in the SFR_DDRCFG register.
	 */
	pmc_sam9x5_enable_periph_clk(AT91C_ID_SFR);
	reg = readl(AT91C_BASE_SFR + SFR_DDRCFG);
	reg |= AT91C_DDRCFG_FDQIEN;
	reg |= AT91C_DDRCFG_FDQSIEN;
	writel(reg, AT91C_BASE_SFR + SFR_DDRCFG);

	/* MPDDRC I/O Calibration Register */
	reg = readl(AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR);
	reg &= ~AT91C_MPDDRC_RDIV;
	reg |= AT91C_MPDDRC_RDIV_DDR2_RZQ_50;
	reg &= ~AT91C_MPDDRC_TZQIO;
	reg |= AT91C_MPDDRC_TZQIO_(100);
	writel(reg, AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR);

	writel(AT91C_MPDDRC_RD_DATA_PATH_ONE_CYCLES,
	       AT91C_BASE_MPDDRC + MPDDRC_RD_DATA_PATH);

	lpddr1_sdram_initialize(AT91C_BASE_MPDDRC,
			        AT91C_BASE_DDRCS, &ddramc_reg);

	ddramc_dump_regs(AT91C_BASE_MPDDRC);
}

#elif defined(CONFIG_LPDDR2)
static void lpddr2_reg_config(struct ddramc_register *ddramc_config)
{
	ddramc_config->mdr = (AT91C_DDRC2_DBW_32_BITS |
			      AT91C_DDRC2_MD_LPDDR2_SDRAM);

	ddramc_config->cr = (AT91C_DDRC2_NC_DDR10_SDR9 |
			     AT91C_DDRC2_NR_14 |
			     AT91C_DDRC2_CAS_3 |
			     AT91C_DDRC2_ZQ_SHORT |
			     AT91C_DDRC2_NB_BANKS_8 |
			     AT91C_DDRC2_UNAL_SUPPORTED);

	ddramc_config->lpddr2_lpr = AT91C_LPDDRC2_DS(0x03);

#ifdef CONFIG_BUS_SPEED_166MHZ
	/*
	 * The MT42128M32 refresh window: 32ms
	 * Required number of REFRESH commands(MIN): 8192
	 * (32ms / 8192) * 166MHz = 0x288.
	 */
	ddramc_config->rtr = 0x288;
	/* 90n short calibration: ZQCS */
	ddramc_config->tim_calr = AT91C_DDRC2_ZQCS(12);

	ddramc_config->t0pr = (AT91C_DDRC2_TRAS_(7) |
			       AT91C_DDRC2_TRCD_(3) |
			       AT91C_DDRC2_TWR_(3) |
			       AT91C_DDRC2_TRC_(11) |
			       AT91C_DDRC2_TRP_(4) |
			       AT91C_DDRC2_TRRD_(2) |
			       AT91C_DDRC2_TWTR_(2) |
			       AT91C_DDRC2_TMRD_(3));

	ddramc_config->t1pr = (AT91C_DDRC2_TRFC_(35) |
				AT91C_DDRC2_TXSNR_(37) |
				AT91C_DDRC2_TXSRD_(0) |
				AT91C_DDRC2_TXP_(2));

	ddramc_config->t2pr = (AT91C_DDRC2_TXARD_(0) |
			       AT91C_DDRC2_TXARDS_(0) |
			       AT91C_DDRC2_TRPA_(0) |
			       AT91C_DDRC2_TRTP_(2) |
			       AT91C_DDRC2_TFAW_(9));
#else
#error "No CLK setting defined"
#endif
}

static void lpddr2_init(void)
{
	struct ddramc_register ddramc_reg;
	unsigned int reg;

	pmc_enable_periph_clock(AT91C_ID_MPDDRC);
	pmc_enable_system_clock(AT91C_PMC_DDR);

	reg = readl(AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR);
	reg &= ~AT91C_MPDDRC_RDIV;
	reg &= ~AT91C_MPDDRC_TZQIO;
	reg |= AT91C_MPDDRC_RDIV_LPDDR2_RZQ_48;
	reg |= AT91C_MPDDRC_TZQIO_(100);
	writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR));

	writel(AT91C_MPDDRC_RD_DATA_PATH_THREE_CYCLES,
	       AT91C_BASE_MPDDRC + MPDDRC_RD_DATA_PATH);

	lpddr2_reg_config(&ddramc_reg);

	lpddr2_sdram_initialize(AT91C_BASE_MPDDRC,
				AT91C_BASE_DDRCS, &ddramc_reg);
}

#elif defined(CONFIG_LPDDR3)
static void lpddr3_reg_config(struct ddramc_register *ddramc_config)
{
	ddramc_config->mdr = (AT91C_DDRC2_DBW_32_BITS |
			      AT91C_DDRC2_MD_LPDDR3_SDRAM);

	ddramc_config->cr = (AT91C_DDRC2_NC_DDR10_SDR9 |
			     AT91C_DDRC2_NR_14 |
			     AT91C_DDRC2_CAS_3 |
			     AT91C_DDRC2_ZQ_INIT |
			     AT91C_DDRC2_NB_BANKS_8 |
			     AT91C_DDRC2_DECOD_SEQUENTIAL |
			     AT91C_DDRC2_UNAL_SUPPORTED);

	ddramc_config->lpddr2_lpr = AT91C_LPDDRC2_DS(0x04);

#ifdef CONFIG_BUS_SPEED_166MHZ
	/* The low-power DDR3-SDRAM device requires a refresh every 3.9 us.*/
	ddramc_config->rtr = 0x288;

	ddramc_config->t0pr = (AT91C_DDRC2_TRAS_(7) |
			       AT91C_DDRC2_TRCD_(3) |
			       AT91C_DDRC2_TWR_(3) |
			       AT91C_DDRC2_TRC_(11) |
			       AT91C_DDRC2_TRP_(4) |
			       AT91C_DDRC2_TRRD_(2) |
			       AT91C_DDRC2_TWTR_(4) |
			       AT91C_DDRC2_TMRD_(10));

	ddramc_config->t1pr = (AT91C_DDRC2_TRFC_(35) |
			       AT91C_DDRC2_TXSNR_(37) |
			       AT91C_DDRC2_TXSRD_(0) |
			       AT91C_DDRC2_TXP_(2));

	ddramc_config->t2pr = (AT91C_DDRC2_TXARD_(0) |
			       AT91C_DDRC2_TXARDS_(0) |
			       AT91C_DDRC2_TRPA_(0) |
			       AT91C_DDRC2_TRTP_(4) |
			       AT91C_DDRC2_TFAW_(9));
#else
#error "No CLK setting defined"
#endif
}

static void lpddr3_init(void)
{
	struct ddramc_register ddramc_reg;
	unsigned int reg;

	pmc_enable_periph_clock(AT91C_ID_MPDDRC);
	pmc_enable_system_clock(AT91C_PMC_DDR);

	reg = readl(AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR);
	reg &= ~AT91C_MPDDRC_RDIV;
	reg &= ~AT91C_MPDDRC_TZQIO;
	reg |= AT91C_MPDDRC_RDIV_LPDDR3_RZQ_57;
	reg |= AT91C_MPDDRC_TZQIO_(100);
	writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR));

	writel(AT91C_MPDDRC_RD_DATA_PATH_THREE_CYCLES,
	       AT91C_BASE_MPDDRC + MPDDRC_RD_DATA_PATH);

	lpddr3_reg_config(&ddramc_reg);

	lpddr3_sdram_initialize(AT91C_BASE_MPDDRC,
				AT91C_BASE_DDRCS, &ddramc_reg);
}
#else
#error "No right DDR-SDRAM device type provided"
#endif

/**
 * The MSBs [bits 31:16] of the CAN Message RAM for CAN0 and CAN1
 * are configured in 0x210000, instead of the default configuration
 * 0x200000, to avoid conflict with SRAM map for PM.
 */
#define CAN_MESSAGE_RAM_MSB	0x21

void at91_init_can_message_ram(void)
{
	writel(AT91C_CAN0_MEM_ADDR_(CAN_MESSAGE_RAM_MSB) |
	       AT91C_CAN1_MEM_ADDR_(CAN_MESSAGE_RAM_MSB),
	       (AT91C_BASE_SFR + SFR_CAN));
}

#ifdef CONFIG_HW_INIT
void hw_init(void)
{


	/* Disable watchdog */
	at91_disable_wdt();

	/*
	 * while coming from the ROM code, we run on PLLA @ 396 MHz / 132 MHz
	 * so we need to slow down and configure MCKR accordingly.
	 * This is why we have a special flavor of the switching function.
	 */

	/* Switch PCK/MCK on Main Clock output */
	//pmc_cfg_mck_down(BOARD_PRESCALER_MAIN_CLOCK);
	pmc_cfg_mck(BOARD_PRESCALER_MAIN_CLOCK);	//3.8.10 has no pmc_cfg_mck_down

	/* Configure PLLA */
	pmc_cfg_plla(PLLA_SETTINGS);

	/* Initialize PLLA charge pump */
	/* No need: we keep what is set in ROM code */
	//pmc_init_pll(0x3);

	/* Switch MCK on PLLA output */
	pmc_cfg_mck(BOARD_PRESCALER_PLLA);

	/* Enable External Reset */
	writel(AT91C_RSTC_KEY_UNLOCK | AT91C_RSTC_URSTEN,
					AT91C_BASE_RSTC + RSTC_RMR);

#if defined(CONFIG_MATRIX)
	/* Initialize the matrix */
	matrix_init();
#endif
	/* initialize the dbgu */
	initialize_dbgu();

	/* Init timer */
	timer_init();

#if defined(CONFIG_DDR3)
	/* Initialize MPDDR Controller */
	ddramc_init();
#elif defined(CONFIG_LPDDR1)
	lpddr1_init();
#elif defined(CONFIG_LPDDR2)
	lpddr2_init();
#elif defined(CONFIG_LPDDR3)
	lpddr3_init();
#endif
	/* Prepare L2 cache setup */
	l2cache_prepare();

	at91_init_can_message_ram();


}
#endif /* #ifdef CONFIG_HW_INIT */

#ifdef CONFIG_DATAFLASH
#if defined(CONFIG_SPI)
void at91_spi0_hw_init(void)
{
#if defined(CONFIG_SPI_BUS0)
#if defined(CONFIG_SPI0_IOSET_1)
	const struct pio_desc spi_pins[] = {
		{"SPI0_SPCK",	AT91C_PIN_PA(14), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPI0_MOSI",	AT91C_PIN_PA(15), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPI0_MISO",	AT91C_PIN_PA(16), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPI0_NPCS",	CONFIG_SYS_SPI_PCS, 1, PIO_DEFAULT, PIO_OUTPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#elif defined(CONFIG_SPI0_IOSET_2)
	const struct pio_desc spi_pins[] = {
		{"SPI0_SPCK",	AT91C_PIN_PB(1), 0, PIO_DEFAULT, PIO_PERIPH_C},
		{"SPI0_MOSI",	AT91C_PIN_PB(0), 0, PIO_DEFAULT, PIO_PERIPH_C},
		{"SPI0_MISO",	AT91C_PIN_PA(31), 0, PIO_DEFAULT, PIO_PERIPH_C},
		{"SPI0_NPCS",	CONFIG_SYS_SPI_PCS, 1, PIO_DEFAULT, PIO_OUTPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#else
#error "No SPI0 IOSET defined"
#endif

#elif defined(CONFIG_SPI_BUS1)

#if defined(CONFIG_SPI1_IOSET_1)
	const struct pio_desc spi_pins[] = {
		{"SPI1_SPCK",	AT91C_PIN_PC(1), 0, PIO_DEFAULT, PIO_PERIPH_D},
		{"SPI1_MOSI",	AT91C_PIN_PC(2), 0, PIO_DEFAULT, PIO_PERIPH_D},
		{"SPI1_MISO",	AT91C_PIN_PC(3), 0, PIO_DEFAULT, PIO_PERIPH_D},
		{"SPI1_NPCS",	CONFIG_SYS_SPI_PCS, 1, PIO_DEFAULT, PIO_OUTPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#elif defined(CONFIG_SPI1_IOSET_2)
	const struct pio_desc spi_pins[] = {
		{"SPI1_SPCK",	AT91C_PIN_PA(22), 0, PIO_DEFAULT, PIO_PERIPH_D},
		{"SPI1_MOSI",	AT91C_PIN_PA(23), 0, PIO_DEFAULT, PIO_PERIPH_D},
		{"SPI1_MISO",	AT91C_PIN_PA(24), 0, PIO_DEFAULT, PIO_PERIPH_D},
		{"SPI1_NPCS",	CONFIG_SYS_SPI_PCS, 1, PIO_DEFAULT, PIO_OUTPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#elif defined(CONFIG_SPI1_IOSET_3)
	const struct pio_desc spi_pins[] = {
		{"SPI1_SPCK",	AT91C_PIN_PD(25), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPI1_MOSI",	AT91C_PIN_PD(26), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPI1_MISO",	AT91C_PIN_PD(27), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SPI1_NPCS",	CONFIG_SYS_SPI_PCS, 1, PIO_DEFAULT, PIO_OUTPUT},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#else
#error "No SPI1 IOSET defined"
#endif
#else
#error "No SPI Bus defined"
#endif

	pio_configure(spi_pins);

	pmc_sam9x5_enable_periph_clk(CONFIG_SYS_ID_SPI);
}
#endif

#if defined (CONFIG_QSPI)
void at91_qspi_hw_init(void)
{
#if defined(CONFIG_QSPI_BUS0)
#if defined(CONFIG_QSPI0_IOSET_1)
	const struct pio_desc qspi_pins[] = {
		{"QSPI0_SCK",	AT91C_PIN_PA(0), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{"QSPI0_CS",	AT91C_PIN_PA(1), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{"QSPI0_IO0",	AT91C_PIN_PA(2), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{"QSPI0_IO1",	AT91C_PIN_PA(3), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{"QSPI0_IO2",	AT91C_PIN_PA(4), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{"QSPI0_IO3",	AT91C_PIN_PA(5), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#elif defined(CONFIG_QSPI0_IOSET_2)
	const struct pio_desc qspi_pins[] = {
		{"QSPI0_SCK",	AT91C_PIN_PA(14), 0, PIO_DEFAULT, PIO_PERIPH_C},
		{"QSPI0_CS",	AT91C_PIN_PA(15), 0, PIO_DEFAULT, PIO_PERIPH_C},
		{"QSPI0_IO0",	AT91C_PIN_PA(16), 0, PIO_DEFAULT, PIO_PERIPH_C},
		{"QSPI0_IO1",	AT91C_PIN_PA(17), 0, PIO_DEFAULT, PIO_PERIPH_C},
		{"QSPI0_IO2",	AT91C_PIN_PA(18), 0, PIO_DEFAULT, PIO_PERIPH_C},
		{"QSPI0_IO3",	AT91C_PIN_PA(19), 0, PIO_DEFAULT, PIO_PERIPH_C},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#elif defined(CONFIG_QSPI0_IOSET_3)
	const struct pio_desc qspi_pins[] = {
		{"QSPI0_SCK",	AT91C_PIN_PA(22), 0, PIO_DEFAULT, PIO_PERIPH_F},
		{"QSPI0_CS",	AT91C_PIN_PA(23), 0, PIO_DEFAULT, PIO_PERIPH_F},
		{"QSPI0_IO0",	AT91C_PIN_PA(24), 0, PIO_DEFAULT, PIO_PERIPH_F},
		{"QSPI0_IO1",	AT91C_PIN_PA(25), 0, PIO_DEFAULT, PIO_PERIPH_F},
		{"QSPI0_IO2",	AT91C_PIN_PA(26), 0, PIO_DEFAULT, PIO_PERIPH_F},
		{"QSPI0_IO3",	AT91C_PIN_PA(27), 0, PIO_DEFAULT, PIO_PERIPH_F},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#else
#error "No QSPI0 IOSET defined"
#endif

#elif defined(CONFIG_QSPI_BUS1)

#if defined(CONFIG_QSPI1_IOSET_1)
	const struct pio_desc qspi_pins[] = {
		{"QSPI1_SCK",	AT91C_PIN_PA(6),  0, PIO_DEFAULT, PIO_PERIPH_B},
		{"QSPI1_CS",	AT91C_PIN_PA(11), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{"QSPI1_IO0",	AT91C_PIN_PA(7),  0, PIO_DEFAULT, PIO_PERIPH_B},
		{"QSPI1_IO1",	AT91C_PIN_PA(8),  0, PIO_DEFAULT, PIO_PERIPH_B},
		{"QSPI1_IO2",	AT91C_PIN_PA(9),  0, PIO_DEFAULT, PIO_PERIPH_B},
		{"QSPI1_IO3",	AT91C_PIN_PA(10), 0, PIO_DEFAULT, PIO_PERIPH_B},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#elif defined(CONFIG_QSPI1_IOSET_2)
	const struct pio_desc qspi_pins[] = {
		{"QSPI1_SCK",	AT91C_PIN_PB(5),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_CS",	AT91C_PIN_PB(6),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO0",	AT91C_PIN_PB(7),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO1",	AT91C_PIN_PB(8),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO2",	AT91C_PIN_PB(9),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO3",	AT91C_PIN_PB(10), 0, PIO_DEFAULT, PIO_PERIPH_D},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#elif defined(CONFIG_QSPI1_IOSET_3)
	const struct pio_desc qspi_pins[] = {
		{"QSPI1_SCK",	AT91C_PIN_PB(14), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"QSPI1_CS",	AT91C_PIN_PB(15), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"QSPI1_IO0",	AT91C_PIN_PB(16), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"QSPI1_IO1",	AT91C_PIN_PB(17), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"QSPI1_IO2",	AT91C_PIN_PB(18), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"QSPI1_IO3",	AT91C_PIN_PB(19), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#else
#error "No QSPI1 IOSET defined"
#endif

#else
#error "No QSPI Bus defined"
#endif

	pio_configure(qspi_pins);

	pmc_sam9x5_enable_periph_clk(CONFIG_SYS_ID_QSPI);
}
#endif
#endif

#ifdef CONFIG_SDCARD
#ifdef CONFIG_OF_LIBFDT
void at91_board_set_dtb_name(char *of_name)
{
	strcpy(of_name, "muxlab_m500768.dtb");
}
#endif


#define ATMEL_SDHC_GCKDIV_VALUE		1

void at91_sdhc_hw_init(void)
{
#ifdef CONFIG_SDHC0
	const struct pio_desc sdmmc_pins[] = {
		{"SDMMC0_CK",   AT91C_PIN_PA(0), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_CMD",  AT91C_PIN_PA(1), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT0", AT91C_PIN_PA(2), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT1", AT91C_PIN_PA(3), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT2", AT91C_PIN_PA(4), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT3", AT91C_PIN_PA(5), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT4", AT91C_PIN_PA(6), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT5", AT91C_PIN_PA(7), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT6", AT91C_PIN_PA(8), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT7", AT91C_PIN_PA(9), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_RSTN", AT91C_PIN_PA(10), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_VDDSEL", AT91C_PIN_PA(11), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_CD",   AT91C_PIN_PA(13), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#endif


#ifdef CONFIG_SDHC1
#error no sdmmc
	const struct pio_desc sdmmc_pins[] = {
		{"SDMMC1_CD",	AT91C_PIN_PA(30), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"SDMMC1_CMD",	AT91C_PIN_PA(28), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"SDMMC1_CK",	AT91C_PIN_PA(22), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"SDMMC1_DAT0",	AT91C_PIN_PA(18), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"SDMMC1_DAT1",	AT91C_PIN_PA(19), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"SDMMC1_DAT2",	AT91C_PIN_PA(20), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"SDMMC1_DAT3",	AT91C_PIN_PA(21), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#endif

	pio_configure(sdmmc_pins);

	pmc_sam9x5_enable_periph_clk(CONFIG_SYS_ID_SDHC);
	pmc_enable_periph_generated_clk(CONFIG_SYS_ID_SDHC,
					GCK_CSS_UPLL_CLK,
					ATMEL_SDHC_GCKDIV_VALUE);
}
#endif

#if defined(CONFIG_TWI0)
unsigned int at91_twi0_hw_init(void)
{
	unsigned int base_addr = AT91C_BASE_TWI0;

	const struct pio_desc twi_pins[] = {
		{"TWD0", AT91C_PIN_PD(29), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{"TWCK0", AT91C_PIN_PD(30), 0, PIO_DEFAULT, PIO_PERIPH_E},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pio_configure(twi_pins);

	pmc_sam9x5_enable_periph_clk(AT91C_ID_TWI0);

	return base_addr;
}
#endif

#if defined(CONFIG_TWI1)
unsigned int at91_twi1_hw_init(void)
{
	const struct pio_desc twi_pins[] = {
		{"TWD1", AT91C_PIN_PD(4), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"TWCK1", AT91C_PIN_PD(5), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pio_configure(twi_pins);

	pmc_sam9x5_enable_periph_clk(AT91C_ID_TWI1);

	return AT91C_BASE_TWI1;
}
#endif

#if defined(CONFIG_AUTOCONFIG_TWI_BUS)
void at91_board_config_twi_bus(void)
{
	act8865_twi_bus = 0;
	at24xx_twi_bus = 1;
}
#endif

#if defined(CONFIG_ACT8865_SET_VOLTAGE)
int at91_board_act8865_set_reg_voltage(void)
{
	unsigned char reg, value;
	int ret;

	/* Check ACT8865 I2C interface */
	if (act8865_check_i2c_disabled())
		return 0;

	reg = REG4_0;
	value = ACT8865_2V5;
	ret = act8865_set_reg_voltage(reg, value);
	if (ret)
		dbg_loud("ACT8865: Failed to make REG4 output 2500mV\n");

	/* Enable REG5 output 3.3V */
	reg = REG5_0;
	value = ACT8865_3V3;
	ret = act8865_set_reg_voltage(reg, value);
	if (ret)
		dbg_loud("ACT8865: Failed to make REG5 output 3300mV\n");

	/* Enable REG6 output 2.5V */
	reg = REG6_0;
	value = ACT8865_2V5;
	ret = act8865_set_reg_voltage(reg, value);
	if (ret)
		dbg_loud("ACT8865: Failed to make REG6 output 2500mV\n");

	/* Enable REG7 output 1.8V */
	reg = REG7_0;
	value = ACT8865_1V8;
	ret = act8865_set_reg_voltage(reg, value);
	if (ret)
		dbg_loud("ACT8865: Failed to make REG7 output 1800mV\n");

	/* Enable REG2 output 1.2V */
	reg = REG2_1;
	value = ACT8865_1V2;
	ret = act8865_set_reg_voltage(reg, value);
	if (ret)
		dbg_loud("ACT8865: Failed to make REG2 output 1200mV\n");

	/* Enable REG4 output 2.5V */
	return 0;
}
#endif


void hw_config_late(void)
{
int timeout;
unsigned int mdio_page;
unsigned int mdio_addr;
unsigned int mdio_data;

unsigned char	data[16];
unsigned int bus;

unsigned char  twi_data;
unsigned char  twi_addr;
unsigned char  twi_dev;
unsigned char  cfg;

	// init M500768 hw specific
	init_si5351b();

// 500768 only
//	AQR105_hw_reset();
	//RTL8307H_hw_reset();
	HDMI_hw_reset();
	FPGA_hw_reset();

	LED_RESET();
	FPGA_reload();
	udelay(200000);	// wait for FPGA to finish loading
	
// 500768 board
//	pio_set_gpio_output(LED_VIDEO, 0);
//	pio_set_gpio_output(LED_ACT, 0);
//	pio_set_gpio_output(LED_LINK, 0);

// 500774 board	
	pio_set_value(LED_VIDEO, 0);
	pio_set_value(LED_ACT, 0);
	pio_set_value(LED_LINK, 0);

	// read DIP switch value
	cfg = read_dipsw();
	dbg_info("DIP SW: %x \n", cfg&0x0F);
	dbg_info("RX_SELECT: %x \n", (cfg>>4)&0x03);
	if(wait_FPGA_done() == 0){
		dbg_info("FPGA reload timeout error.\n");
	}
	
// 500768 only
//	pwm_init();	
	
// MDIO(0x64) Currecntly removed from FPGA
#if 0 // 500768 board
#if 1
	timeout = 1000 / 50;	// 5 sec delay max
	mdio_page=0x1;  mdio_addr=0xcc02;
	mdio_write_addr(AQR105_PRTAD, mdio_page, mdio_addr);	 // set reg addr

	while (--timeout!=0) {
		mdio_read_data(AQR105_PRTAD, mdio_page, &mdio_data);	// read data
		if ((mdio_data & 0x03) == 0x01) break;		
		udelay(50000);
		dbg_info(".");
	}
#endif
	mdio_page=0x1e;  mdio_addr=0x0020;
	mdio_write_addr(AQR105_PRTAD, mdio_page, mdio_addr);	 // set reg addr
	mdio_read_data(AQR105_PRTAD, mdio_page, &mdio_data);	// read data
	dbg_info("AQR105 Firmware version %x.%x\n", (mdio_data>>8), (mdio_data&0xff) );
#else // 500774 board
#endif


	bus = 0;
	twi_dev = (0x60>>1);
	twi_addr = 0;

	// sw reset fpga global
	twi_data = 0x80;	// reset FPGA
	// Assert global reset
	twi_write(bus, twi_dev, twi_addr, 1, &twi_data, 1);	
	twi_data = 0x0;	// clear reset
	// De-assert global reset
	twi_write(bus, twi_dev, twi_addr, 1, &twi_data, 1);

	// sw reset sub modules
	twi_data = 0x7f;	// reset all FPGA sub modules
	// Assert reset to each FPGA sub-system: XAUI, Video, PCS/PMA, 1GB Ethernet, HDMI/Microblaze
	twi_write(bus, twi_dev, twi_addr, 1, &twi_data, 1);
	twi_data = 0x0;		// clear reset
	// De-assert reset to each FPGA sub-system: XAUI, Video, PCS/PMA, 1GB Ethernet, HDMI/Microblaze
	twi_write(bus, twi_dev, twi_addr, 1, &twi_data, 1);

// reset PCS/PMA
#if 1 // 500774 board
	twi_dev = (0x6E>>1);
	twi_addr = 0;
	twi_data = 0x12;		// set reset
	// Assert reset to PCS and PMA
	twi_write(bus, twi_dev, twi_addr, 1, &twi_data, 1);
	twi_data = 0x0;		// clear reset
	// De-assert reset to PCS and PMA
	twi_write(bus, twi_dev, twi_addr, 1, &twi_data, 1);
	
	twi_dev = (0x60>>1);
#endif

	// disable all stream (video, audio, etc)
	twi_addr = 3;
	twi_data = 0x0;
	// Disable all streams: Video, Audio, SIO, 1GB Ethernet, Ancillary, and Auxiliary	
	twi_write(bus, twi_dev, twi_addr, 1, &twi_data, 1);

	//read reg 03 value, if not 0, disable it again.
	twi_read(bus, twi_dev, twi_addr, 1, &twi_data, 1);
	if(twi_data != 0x0)
	{
		twi_data = 0x0;
		twi_addr = 3;	
		twi_write(bus, twi_dev, twi_addr, 1, &twi_data, 1);
		dbg_info("set reg 03 to 0 again %x\n");
	}
	else
		dbg_info("read 03 reg %x\n", twi_data);


	// read fpga version,revision
	twi_addr = 29;
	twi_read(bus, twi_dev, twi_addr, 1, &data, 3);
	dbg_info("FPGA version %x %x %x\n", data[0],data[1],data[2] );


	if ((data[0] & 0x80) == 0) {
		dbg_info("FPGA RX unit \n");
		dac_param(1000);
		fpag_init_rx();

		dbg_info("HDMI RX unit \n");
		init_video_param();

	} else {		
		set_tx_si5351b();
		dbg_info("FPGA TX unit \n");
		dac_param(0);
		fpag_init_tx();
		
		dbg_info("HDMI TX unit \n");
		init_hdmi();
		
	}



//	while (1) {
//		ADIAPI_RxHouseKeeping ();
//		udelay(500);
//	}
}


#define MCP4706_BUS   1

void  dac_param(int  dac)
{
unsigned char	twi_dev;
unsigned char	twi_addr;
unsigned char	data[16];
unsigned int bus = 0;
unsigned char	cmd;
unsigned char	pd;
unsigned int  udac;

	udac = 0x7ff + dac;
	bus = 0;
	data[0] = 0x04 | MCP4706_BUS;	// set mux for si5351b 
	twi_write(bus, PCA9544_ADDR, 0, 0, data, 1);

	twi_dev = (0XC0>>1);
	cmd = 0;
	pd = 0;
	data[0] = ((cmd&0X03) << 6) | ((pd&0X03)<<4) | ((udac>>8)&0x0f);
	data[1] = udac;
	twi_write(bus, twi_dev, 0, 0, data, 2);
}

/*============================================================================
 * Initialize devices for HDMI Input operation
 * Tx side only
 * 
 * 
 *===========================================================================*/
void init_hdmi (void)
{

}

/*============================================================================
 * Initialize devices for HDMI Output operation
 * Rx side only
 * 
 * 
 *===========================================================================*/

void  init_video_param(void)
{
unsigned char	twi_dev;
unsigned char	twi_addr;
unsigned char	data[16];
unsigned int bus = 0;
//unsigned int chan = 0;

unsigned int x, y;
unsigned int sync_x, sync_y;

unsigned int sx, sy;

#if 0
#if 1
	x = 3840;
	y = 2160;
	sync_x = 560;
	sync_y = 90; //48;

	Sil9136_set_video_info(29700, 30, (x+sync_x), (y+sync_y-1));

	init_pll_drp(0);	// X 2 

#else
	x = 1920;
	y = 1080;
	sync_x = 280;
	sync_y = 45;

	Sil9136_set_video_info(14850, 60, (x+sync_x), (y+sync_y-1));

	init_pll_drp(1);	// X 1

#endif
#endif


	twi_dev = (0x60>>1);

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

	// FPGA does not have video offset registers ???
	/*
	twi_addr = 14;
	sx = sync_x-1;
	sy = sync_y-1;
	// video offset
	data[0] = (sx & 0xff);
	data[1] = (sx >> 8);
	data[2] = (sy & 0xff);
	data[3] = (sy >> 8);
	twi_write(bus, twi_dev, twi_addr, 1, data, 4);
	*/

	return;
}

// PLL Initialization
void  init_pll_drp(unsigned char mode)
{
unsigned char	twi_dev;
unsigned char	twi_addr;
unsigned char	data[16];
unsigned int bus = 0;
//unsigned int chan = 0;

unsigned char	drp_daddr;
unsigned drp_data;

unsigned char	cycle;

	switch (mode) {
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
//	twi_write(bus, twi_dev, twi_addr, 1, data, 5);
	twi_write(bus, twi_dev, twi_addr, 1, data, 4);

	twi_addr = 4;
	twi_read(bus, twi_dev, twi_addr, 1, data, 3);

	dbg_info("DRP read %x %x %x\n", data[0], data[1], data[2] );


	return;
}

/*============================================================================
 * Initialize FPGA
 * Tx side only
 * 
 * 
 *===========================================================================*/
void fpag_init_tx(void)
{
unsigned char  bus;
unsigned char  twi_data;
unsigned char  twi_addr;
unsigned char  twi_dev;
unsigned char	data[16];

	bus = 0;
	twi_dev = (0x60>>1);
	twi_addr = 1;
	twi_data = (7<<2) | (0<<0);
	// Set the video format as: bits(4:2) = RGB Raw, bits(1:0) = 8-bits per pixel
	twi_write(bus, twi_dev, twi_addr, 1, &twi_data, 1);

  // for debug
	twi_addr = 0;
	data[0] = 0xff;
	data[1] = 0xfe;
	data[2] = 0xfd;
	data[3] = 0xfc;
	twi_read(bus, twi_dev, twi_addr, 1, &data, 4);
	dbg_info("FPGA(60.0) %x %x %x %x \n", data[0],data[1],data[2], data[3] );

	twi_dev = (0x68>>1);
	twi_addr = 4;
	data[0] = 0x10;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0xE5;
	data[4] = 0xA0;
	data[5] = 0x00;
	// Set the 48-bit Local MAC address 
	twi_write(bus, twi_dev, twi_addr, 1, &data, 6);

	twi_addr = 10;
	data[0] = 0x11;
	// Set the 48-bit Destination MAC address
	twi_write(bus, twi_dev, twi_addr, 1, &data, 6);

	twi_addr = 16;
	data[0] = 210;
	data[1] = 4;
	data[2] = 0;
	data[3] = 10;
	// Set the 32-bit Local Video IP address 
	twi_write(bus, twi_dev, twi_addr, 1, &data, 4);

// video dest ip
	twi_addr = 20;
	data[0] = 211;
	data[1] = 4;
	data[2] = 0;
	data[3] = 10;
	// Set the 32-bit Destination Video IP address  
	twi_write(bus, twi_dev, twi_addr, 1, &data, 4);

	twi_addr = 32;
	// Set the 16-bit Local UDP Video Port 
	data[0] = 0x30;
	data[1] = 0xc2;
	// Set the 16-bit Destination UDP Video Port
	data[2] = 0x20;
	data[3] = 0xc2;
	twi_write(bus, twi_dev, twi_addr, 1, &data, 4);

	twi_addr = 36;
	// Set the 16-bit Local UDP Audio Port 
	data[0] = 0x50;
	data[1] = 0xc2;
	// Set the 16-bit Destination UDP Audio Port
	data[2] = 0x40;
	data[3] = 0xc2;
	twi_write(bus, twi_dev, twi_addr, 1, &data, 4);

	twi_addr = 40;
	// Set the 16-bit Local UDP Ancillary Port 
	data[0] = 0x70;
	data[1] = 0xc2;
	// Set the 16-bit Destination UDP Ancillary Port
	data[2] = 0x60;
	data[3] = 0xc2;
	twi_write(bus, twi_dev, twi_addr, 1, &data, 4);

//	twi_addr = 1;
//	data[0] = 0x06;  // enable audio + video
//	twi_write(bus, twi_dev, twi_addr, 1, &data, 1);


}

/*============================================================================
 * Initialize FPGA
 * Rx side only
 * 
 * 
 *===========================================================================*/
void fpag_init_rx(void)
{
unsigned char  bus;
unsigned char  twi_data;
unsigned char  twi_addr;
unsigned char  twi_dev;
unsigned char	data[16];

	bus = 0;
	twi_dev = (0x60>>1);

	twi_addr = 1;
	twi_data = (7<<2) | (0<<0);
	// Set the video format as: bits(4:2) = RGB Raw, bits(1:0) = 8-bits per pixel
	twi_write(bus, twi_dev, twi_addr, 1, &twi_data, 1);

  // for debug
	twi_addr = 0;
	data[0] = 0xff;
	data[1] = 0xfe;
	data[2] = 0xfd;
	data[3] = 0xfc;
	twi_read(bus, twi_dev, twi_addr, 1, &data, 4);
	dbg_info("FPGA(60.0) %x %x %x %x \n", data[0],data[1],data[2], data[3] );

	twi_dev = (0x68>>1);
	twi_addr = 4;
	data[0] = 0x11; //10;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0xE5;
	data[4] = 0xA0;
	data[5] = 0x00;
	// Set the 48-bit Local MAC address 
	twi_write(bus, twi_dev, twi_addr, 1, &data, 6);

	twi_addr = 10;
	data[0] = 0x10; //0x11;
	// Set the 48-bit Destination MAC address
	twi_write(bus, twi_dev, twi_addr, 1, &data, 6);

	twi_addr = 16;
	data[0] = 211; //210;
	data[1] = 4;
	data[2] = 0;
	data[3] = 10;
	// Set the 32-bit Local Video IP address 
	twi_write(bus, twi_dev, twi_addr, 1, &data, 4);

// video dest ip
	twi_addr = 20;
	data[0] = 210; // 211;
	data[1] = 4;
	data[2] = 0;
	data[3] = 10;
	// Set the 32-bit Destination Video IP address  
	twi_write(bus, twi_dev, twi_addr, 1, &data, 4);

	twi_addr = 32;
	// Set the 16-bit Local UDP Video Port 
	data[0] = 0x30;
	data[1] = 0xc2;
	// Set the 16-bit Destination UDP Video Port
	data[2] = 0x20;
	data[3] = 0xc2;
	twi_write(bus, twi_dev, twi_addr, 1, &data, 4);

	twi_addr = 36;
	// Set the 16-bit Local UDP Audio Port 
	data[0] = 0x50;
	data[1] = 0xc2;
	// Set the 16-bit Destination UDP Audio Port
	data[2] = 0x40;
	data[3] = 0xc2;
	twi_write(bus, twi_dev, twi_addr, 1, &data, 4);

	twi_addr = 40;
	// Set the 16-bit Local UDP Ancillary Port 
	data[0] = 0x70;
	data[1] = 0xc2;
	// Set the 16-bit Destination UDP Ancillary Port
	data[2] = 0x60;
	data[3] = 0xc2;
	twi_write(bus, twi_dev, twi_addr, 1, &data, 4);

//	twi_addr = 1;
//	data[0] = 0x06;  // enable audio + video
//	twi_write(bus, twi_dev, twi_addr, 1, &data, 1);


}

