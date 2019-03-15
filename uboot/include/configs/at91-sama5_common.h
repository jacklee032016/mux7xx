/*
 * Common part of configuration settings for the AT91 SAMA5 board.
 *
 * Copyright (C) 2015 Atmel Corporation
 *		      Josh Wu <josh.wu@atmel.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __AT91_SAMA5_COMMON_H
#define __AT91_SAMA5_COMMON_H

#include <asm/hardware.h>

#define CONFIG_SYS_TEXT_BASE		0x26f00000

/* ARM asynchronous clock */
#define CONFIG_SYS_AT91_SLOW_CLOCK      32768
#define CONFIG_SYS_AT91_MAIN_CLOCK      12000000 /* from 12 MHz crystal */

#define CONFIG_ARCH_CPU_INIT

#ifndef CONFIG_SPL_BUILD
#define CONFIG_SKIP_LOWLEVEL_INIT
#endif

#define CONFIG_ENV_VARS_UBOOT_CONFIG

/* general purpose I/O */
#ifndef CONFIG_DM_GPIO
#define CONFIG_AT91_GPIO
#endif


/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME

/*
 * Command line configuration.
 */

#ifdef CONFIG_SYS_USE_MMC

#ifdef CONFIG_ENV_IS_IN_MMC
/* Use raw reserved sectors to save environment */
#define CONFIG_ENV_OFFSET		0x2000
#define CONFIG_ENV_SIZE			0x1000
#define CONFIG_SYS_MMC_ENV_DEV		0
#else
/* u-boot env in sd/mmc card */
#define CONFIG_ENV_IS_IN_FAT
#define CONFIG_FAT_WRITE
#define FAT_ENV_INTERFACE	"mmc"
#define FAT_ENV_DEVICE_AND_PART	"0"
#define FAT_ENV_FILE		"uboot.env"
#define CONFIG_ENV_SIZE		0x4000
#endif

#define CONFIG_BOOTCOMMAND	"if test ! -n ${dtb_name}; then "	\
				    "setenv dtb_name at91-${board_name}.dtb; " \
				"fi; "					\
				"fatload mmc 0:1 0x21000000 ${dtb_name}; " \
				"fatload mmc 0:1 0x22000000 zImage; "	\
				"bootz 0x22000000 - 0x21000000"
#define CONFIG_BOOTARGS							\
	"console=ttyS0,115200 earlyprintk "				\
	"root=/dev/mmcblk0p2 rw rootwait"
#else
#define CONFIG_BOOTARGS							\
	"console=ttyS0,115200 earlyprintk "				\
	"mtdparts=atmel_nand:256k(bootstrap)ro,768k(uboot)ro,"		\
	"256K(env_redundant),256k(env),"				\
	"512k(dtb),6M(kernel)ro,-(rootfs) "				\
	"rootfstype=ubifs ubi.mtd=6 root=ubi0:rootfs"

#ifdef CONFIG_SYS_USE_NANDFLASH
/* u-boot env in nand flash */
#define CONFIG_ENV_IS_IN_NAND
#define CONFIG_ENV_OFFSET		0x140000
#define CONFIG_ENV_OFFSET_REDUND	0x100000
#define CONFIG_ENV_SIZE			0x20000
#define CONFIG_BOOTCOMMAND		"nand read 0x21000000 0x180000 0x80000;"	\
					"nand read 0x22000000 0x200000 0x600000;"	\
					"bootz 0x22000000 - 0x21000000"
#elif CONFIG_SYS_USE_SERIALFLASH
/* u-boot env in serial flash, by default is bus 0 and cs 0 */
#define CONFIG_ENV_IS_IN_SPI_FLASH
#ifdef CONFIG_QSPI_BOOT
#define CONFIG_ENV_OFFSET		0x10000
#define CONFIG_ENV_SIZE			0x10000
#define CONFIG_ENV_SECT_SIZE		0x1000
#define CONFIG_BOOTCOMMAND		"sf probe 0; "				\
					"sf read 0x22000000 0xB0000 0x400000; "	\
					"sf read 0x25000000 0x4D0000 0x1400000; "	\
					"sf read 0x21000000 0xA0000 0x10000; "	\
					"bootz 0x22000000 - 0x21000000"
#undef CONFIG_BOOTARGS
/*#define CONFIG_BOOTARGS		"console=ttyS0,115200 ip=10.0.1.233 earlyprintk " \
				"root=/dev/mtdblock5 rootfstype=jffs2 rw rootwait" */
#define CONFIG_BOOTARGS		"initrd=0x25000000,40M root=/dev/ram0 rw init=/linuxrc " \
        "console=ttyS0,115200" 
/*        
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
  "setmac=if setenv ethaddr '00:12:23:34:45:56'; then saveenv; fi\0"
#define CONFIG_ENV_OVERWRITE    1	//enable change MAC with uboot cmd 0502
*/
#else
#define CONFIG_ENV_OFFSET		0x6000
#define CONFIG_ENV_SIZE			0x2000
#define CONFIG_ENV_SECT_SIZE		0x1000
#define CONFIG_BOOTCOMMAND		"sf probe 0; "				\
					"sf read 0x21000000 0x60000 0xc000; "	\
					"sf read 0x22000000 0x6c000 0x394000; "	\
					"bootz 0x22000000 - 0x21000000"
#endif

#endif

#endif

#define CONFIG_BAUDRATE			115200

#define CONFIG_SYS_CBSIZE		256
#define CONFIG_SYS_MAXARGS		16
#define CONFIG_SYS_LONGHELP
#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(4 * 1024 * 1024)

#endif
