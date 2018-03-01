
/*
 * Copyright (C) 2016 NXP Semiconductors
 *
 * Configuration settings for the i.MX7d Rugo board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __RUGO_H
#define __RUGO_H

#include "mx7_common.h"


#define PHYS_SDRAM_SIZE			SZ_1G

#define CONFIG_MXC_UART_BASE		UART1_IPS_BASE_ADDR

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(35 * SZ_1M)


/* #define CONFIG_DISPLAY_BOARDINFO */

/* I2C configs */

#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1
#define CONFIG_SYS_I2C_SPEED		100000

/* PMIC */

#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE3000
#define CONFIG_POWER_PFUZE3000_I2C_ADDR	0x08

/* MMC Config*/
#define CONFIG_SYS_FSL_ESDHC_ADDR       USDHC3_BASE_ADDR 
#define CONFIG_SUPPORT_EMMC_BOOT
#define CONFIG_SYS_FSL_ESDHC_HAS_DDR_MODE	  			 
#define CONFIG_SYS_MMC_IMG_LOAD_PART	1


#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x20000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR    
#define CONFIG_SYS_HZ			1000      


/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR   

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR 
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE 	   

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_ENV_SIZE			SZ_8K
#define CONFIG_ENV_IS_IN_MMC

#define CONFIG_ENV_OFFSET		(8 * SZ_64K)
#define CONFIG_SYS_FSL_USDHC_NUM	1        //sd and emmc,sd:2,emmc:3

#define CONFIG_SYS_MMC_ENV_DEV		0								
#define CONFIG_SYS_MMC_ENV_PART		0				
#define CONFIG_MMCROOT			"/dev/mmcblk2p2"   // use emmc as the root

/* USB Configs */
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX7
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS  0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

/*#define CONFIG_IMX_THERMA11*/
#define CONFIG_USBD_HS

#define CONFIG_USB_FUNCTION_MASS_STORAGE

/* USB Device Firmware Update support */
#define CONFIG_DFU_MMC
#define CONFIG_SYS_DFU_DATA_BUF_SIZE	SZ_16M
#define DFU_DEFAULT_POLL_TIMEOUT	300

/*#define CONFIG_VIDEO*/
#ifdef CONFIG_VIDEO
#define CONFIG_CFB_CONSOLE
#define CONFIG_VIDEO_MXS
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_SW_CURSOR
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_SYS_CONSOLE_BG_COL            0x00
#define CONFIG_SYS_CONSOLE_FG_COL            0xa0

#endif


#define CONFIG_DFU_ENV_SETTINGS \
	"dfu_alt_info=boot raw 0x2 0x400 mmcpart 1\0" \

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_DFU_ENV_SETTINGS \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"console=ttymxc0\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_file=imx7d-rugo.dtb\0" \
	"fdt_addr=0x83000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"videomode=video=ctfb:x:480,y:272,depth:24,pclk:108695,le:8,ri:4,up:2,lo:4,hs:41,vs:10,sync:0,vmode:0\0" \
	"mmcdev="__stringify(CONFIG_SYS_MMC_ENV_DEV)"\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"mmcargs=setenv bootargs console=${console},${baudrate} " \
		"root=${mmcroot}\0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} " \
		"root=/dev/nfs " \
	"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
		"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev};" \
	   "mmc dev ${mmcdev}; if mmc rescan; then " \
		   "if run loadbootscript; then " \
			   "run bootscript; " \
		   "else " \
			   "if run loadimage; then " \
				   "run mmcboot; " \
			   "else run netboot; " \
			   "fi; " \
		   "fi; " \
	   "else run netboot; fi"

#endif /*__RUGO_H*/
