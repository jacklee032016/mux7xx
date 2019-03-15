/*
 * Configuration file for the SAMA5D2 Xplained Board.
 *
 * Copyright (C) 2015 Atmel Corporation
 *		      Wenyou Yang <wenyou.yang@atmel.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "at91-sama5_common.h"

#define CONFIG_MISC_INIT_R

/* SDRAM */
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_SDRAM_BASE           ATMEL_BASE_DDRCS
#define CONFIG_SYS_SDRAM_SIZE		0x10000000

#ifdef CONFIG_SPL_BUILD
#define CONFIG_SYS_INIT_SP_ADDR		0x218000
#else
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_SDRAM_BASE + 16 * 1024 - GENERATED_GBL_DATA_SIZE)
#endif

#define CONFIG_SYS_LOAD_ADDR		0x22000000 /* load address */

/* SerialFlash */
#ifdef CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_BUS		1
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		40000000
#define  CONFIG_ENV_SPI_MAX_HZ		40000000
#endif
#define CONFIG_SPI_FLASH_BAR	1
/* NAND flash */
#undef CONFIG_CMD_NAND

/* I2C */
#define AT24MAC_ADDR		0x5c
#define AT24MAC_REG		0x9a

/* LCD */
#ifdef CONFIG_ATMEL_HLCD
#define CONFIG_SYS_WHITE_ON_BLACK
#endif

#ifdef CONFIG_SYS_USE_MMC

/* bootstrap + u-boot + env in sd card */
#undef FAT_ENV_DEVICE_AND_PART
#undef CONFIG_BOOTCOMMAND

#define FAT_ENV_DEVICE_AND_PART	"1"
#define CONFIG_BOOTCOMMAND	"sf probe 0; sf read 0x21000000 0xA0000 0x10000; sf read 0x22000000 0xB0000 0x400000; bootz 0x22000000 - 0x21000000"
#undef CONFIG_BOOTARGS
#define CONFIG_BOOTARGS \
	"console=ttyS0,115200 ip=10.0.1.233 earlyprintk root=/dev/mtdblock5 rootfstype=jffs2 rw rootwait"
#define CONFIG_ENV_OVERWRITE    1

#endif

#ifndef CONFIG_SYS_USE_MMC
#ifdef CONFIG_QSPI_BOOT

#define CONFIG_IPADDR 10.0.1.230
#define CONFIG_NETMASK 255.255.255.0
#define CONFIG_GATEWAYIP 10.0.1.1
#define CONFIG_SERVERIP 10.0.1.233
#define CONFIG_EXTRA_ENV_SETTINGS \
  "wrdtbrx=if tftpboot 0x21000000 500774/m500774_rxdtb.dtb; then sf write 0x21000000 0xa0000 0x10000; fi\0" \
  "wrdtbtx=if tftpboot 0x21000000 500774/m500774_txdtb.dtb; then sf write 0x21000000 0xa0000 0x10000; fi\0" \
  "wrzim=if tftpboot 0x21000000 500774/m500774_zImage; then sf write 0x21000000 0xb0000 0x400000; fi\0" \
  "wredid=if tftpboot 0x21000000 500774/m500774_edid_default.bin; then sf write 0x21000000 0x4c0000 0x10000; fi\0" \
  "wrroot=if tftpboot 0x21000000 500774/m500774_ramdisk.gz; then sf write 0x21000000 0x4d0000 0x1400000; fi\0" \
  "wrapptx=if tftpboot 0x21000000 500774/m500774_txapp; then sf write 0x21000000 0x18d0000 0x730000; fi\0" \
  "wrapprx=if tftpboot 0x21000000 500774/m500774_rxapp; then sf write 0x21000000 0x18d0000 0x730000; fi\0" \
  "erall=if sf probe 0; then sf erase 0xa0000 0x1f50000; fi\0" \
  "erapp=if sf probe 0; then sf erase 0x18d0000 0x730000; fi\0" \
  "setmactx=if setenv ethaddr '00:12:23:34:45:01'; then saveenv; fi\0" \
  "setmacrx=if setenv ethaddr '00:12:23:34:45:02'; then saveenv; fi\0" \
  "setmac=if setenv ethaddr '00:12:23:34:45:56'; then saveenv; fi\0"
#define CONFIG_ENV_OVERWRITE    1	//enable change MAC with uboot cmd 0502


#endif
#endif


/* SPL */
#define CONFIG_SPL_FRAMEWORK
#define CONFIG_SPL_TEXT_BASE		0x200000
#define CONFIG_SPL_MAX_SIZE		0x10000
#define CONFIG_SPL_BSS_START_ADDR	0x20000000
#define CONFIG_SPL_BSS_MAX_SIZE		0x80000
#define CONFIG_SYS_SPL_MALLOC_START	0x20080000
#define CONFIG_SYS_SPL_MALLOC_SIZE	0x80000

#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SYS_MONITOR_LEN		(512 << 10)

#ifdef CONFIG_SYS_USE_MMC
#define CONFIG_SPL_LDSCRIPT		arch/arm/mach-at91/armv7/u-boot-spl.lds
#define CONFIG_SYS_MMCSD_FS_BOOT_PARTITION	1
#define CONFIG_SPL_FS_LOAD_PAYLOAD_NAME		"u-boot.img"

#elif CONFIG_SYS_USE_SERIALFLASH
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x20000

#endif

#endif
