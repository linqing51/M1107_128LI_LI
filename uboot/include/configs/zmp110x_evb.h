/*
 * (C) Copyright 2003
 * Texas Instruments.
 * Kshitij Gupta <kshitij@ti.com>
 * Configuation settings for the TI OMAP Innovator board.
 *
 * (C) Copyright 2004
 * ARM Ltd.
 * Philippe Robin, <philippe.robin@arm.com>
 * Configuration for Versatile PB.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* define chip serial */
/* #define ZMP1106_CHIP */
#define ZMP1107_CHIP

#define CONFIG_CMD_MISC
#define CONFIG_SYS_HUSH_PARSER

#if !defined(CONFIG_SPL_BUILD)
#define CONFIG_HW_WATCHDOG
#define CONFIG_HW_WATCHDOG_GPIO 3
#endif

/* define boot device,
 * default boot device is spi-nor,
 * if one machine use spi-nand as boot device,
 * pls uncomment CONFIG_SPINAND_FLASH
 */
 #define CONFIG_SPINAND_FLASH

#ifdef CONFIG_SPINAND_FLASH
#define CONFIG_SPL_BOOT
#endif

#ifdef ZMP1107_CHIP
#define CONFIG_DRAM_128M
#else
#define CONFIG_DRAM_64M
#endif

/*
* enable fdt support
*/
#define CONFIG_OF_CONTROL
#define CONFIG_OF_LIBFDT

/*--------------------------------------chip------------------------------------------------*/

/*
 * High Level Configuration Options
 * (easy to change)
 */
#define CONFIG_ARM926EJS	1	/* This is an arm926ejs CPU core */
#define CONFIG_PLAT_ZMP 	1
#define CONFIG_USE_IRQ      0
#define CONFIG_BOARD_EARLY_INIT_F
/*
#define CONFIG_DELAY_ENVIRONMENT 1  // add for environment delay
*/

/*use for arch/arm/cpu/arm926ejs/start.S */
/*read uboot from 512 bytes offset pos in spi flash 0~511 is irom param.*/
#ifdef  CONFIG_SPINAND_FLASH
#define CONFIG_SYS_TEXT_BASE   0x80eff800
#else
#define CONFIG_SYS_TEXT_BASE   0x80dffe00
#endif

/*
 * Stack sizes
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128*1024)	/* regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(8*1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4*1024)	/* FIQ stack */
#endif

/*
 * input clock of PLL
 */
#define CONFIG_SYS_CLK_FREQ	12000000
#define CONFIG_SYS_HZ  		(1000)

#define CONFIG_ZMP_WATCHDOG   1


/*--------------------------------------device------------------------------------------------*/
/*
* select RMII ethernet interface
*/
#define CONFIG_ETHERNET_MAC_RMII 	1

/*
* macro define switch for uoot logo mipi&rgb lcd display
*/
#define CONFIG_LCD		1
#ifdef CONFIG_LCD
#define CONFIG_LCD_BACKLIGHT_GPIO 40
#define CONFIG_LCD_BACKLIGHT_PWM 0
/* #define CONFIG_LCD_BACKLIGHT_CLOSE 1 */
#define CONFIG_ZMP_LCD	1
#define CONFIG_ZMP_FB
#define CONFIG_ZMP_DP
/*
* init lcd pannel size for request reserved space
*/
#define LCD_XRES			800
#define LCD_YRES			1280
#define LCD_BPP				LCD_COLOR24
#endif

/*
* Flash config
*/
/*#define CONFIG_CMD_FLASH   1  */
#define CONFIG_ZMP_SPI
/*#define CONFIG_SPI_XFER_CPU*/
#define CONFIG_SPI_XFER_DMA
#define CONFIG_SPI_FLASH_BUSWIDTH_4X
#define CONFIG_SPI_FLASH
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH_ZMP 		1
#define CONFIG_SYS_MAX_FLASH_SECT 	8192
#define CONFIG_SYS_MAX_NAND_DEVICE  1
#define CONFIG_SYS_NAND_BASE      0x20120000
#define CONFIG_SPIFLASH_USE_FAST_READ 1
#define CONFIG_SPI_FLASH_BAR		1

/*
 * Hardware drivers
 */
#define CONFIG_ZMP_SERIAL
#define CONFIG_SERIAL1
#define CONFIG_ZMP_ETHER
#define CONFIG_CMD_NET
#define CONFIG_CMD_PING

/* MMC */
#define CONFIG_CMD_FAT    1
#define CONFIG_PARTITIONS 1
#define CONFIG_DOS_PARTITION		1
#define CONFIG_MMC     1
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC		1
#define CONFIG_ZMP_SDHSMMC		1

/* select TF plug in or plug off use GPIO or MCK */
#define TF_DET_MCK_MODE		0xFF

/*--------------------------------------u-boot------------------------------------------------*/

/*
 * Stack should be on the SRAM because
 * DRAM is not init ,(arch/arm/lib/crt0.S)
 */
#define CONFIG_SYS_INIT_SP_ADDR		(0x80ff0000 - GENERATED_GBL_DATA_SIZE)

/*
 * add print version info
 */
#define CONFIG_VERSION_VARIABLE 1

/*
* add for uart download
*/
#define CONFIG_CMD_LOADB


/* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_FLASH_BASE		0x0


/* enviroment variable offset and size in spi nor flash. According to customer's requirement,it can be modified.*/
#ifdef  CONFIG_RTT_DTB_ADDR
#define CONFIG_DTB_LOADADDR			(0x83fd3000)
#else
#define CONFIG_DTB_LOADADDR			(0x81300000)
#endif

#ifdef  CONFIG_SPINAND_FLASH
#define CONFIG_SPL_TEXT_BASE   		0x80fff800
#define CONFIG_SPL_MAX_FOOTPRINT	0x20000 /*32768*/
#else
#define CONFIG_SPL_TEXT_BASE   		0x80effe00
#define CONFIG_SPL_MAX_FOOTPRINT	0x20000 /*32768*/
#endif

#ifdef  CONFIG_SPINAND_FLASH /* spi-nand */
#define CONFIG_ENV_OFFSET  			(0x20000)
#define CONFIG_ENV_SIZE  			(0x2000)
#define CONFIG_BKENV_OFFSET  		(0x40000)
#define CONFIG_BKENV_SIZE  			(0x2000)
#define CONFIG_ENV_PARTITION_SIZE  	(128*1024)
#define CONFIG_DTB_OFFSET  			(0x60000)
#define CONFIG_DTB_SIZE  			(64*1024)
#define CONFIG_DTB_FDTLADDR			(CONFIG_SPL_TEXT_BASE+CONFIG_DTB_OFFSET)
#else

#ifdef CONFIG_SPL_BOOT   /*spl*/
#define CONFIG_ENV_OFFSET  			(0x10000)
#define CONFIG_ENV_SIZE  			(4*1024)
#define CONFIG_BKENV_OFFSET  		(0x11000)
#define CONFIG_BKENV_SIZE  			(4*1024)
#define CONFIG_DTB_OFFSET  			(0x12000)
#define CONFIG_DTB_SIZE  			(64*1024)
#define CONFIG_DTB_FDTLADDR			(CONFIG_SPL_TEXT_BASE+CONFIG_DTB_OFFSET)

#else  /*uboot*/
#define CONFIG_ENV_OFFSET  			(0x40000)
#define CONFIG_ENV_SIZE  			(4*1024)
#define CONFIG_BKENV_OFFSET  		(0x50000)
#define CONFIG_BKENV_SIZE  			(4*1024)
#define CONFIG_DTB_OFFSET  			(0x60000)
#define CONFIG_DTB_SIZE  			(64*1024)
#define CONFIG_DTB_FDTLADDR			(CONFIG_SYS_TEXT_BASE+CONFIG_DTB_OFFSET)
#endif
#endif
/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN	(CONFIG_ENV_SIZE + 4096*1024)

/* ATAG configuration */
#define CONFIG_CMDLINE_TAG
#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE

/*
 * Miscellaneous configurable options
 */
#define CONFIG_LONGHELP
#define CONFIG_SYS_LONGHELP	1
#define CONFIG_SYS_PROMPT	"\nzmp110x > "
#define CONFIG_SYS_CBSIZE	512
#define CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16)
#define CONFIG_SYS_MAXARGS	32
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE

/* default load address	*/
#define CONFIG_SYS_LOAD_ADDR		0x82008000

/*
 * Check if key already pressed
 * Don't check if bootdelay < 0
 */
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_BOOTDELAY 3

#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_AUTOBOOT_PROMPT "\nHit zlg to stop autoboot:%d * 100 ms\n",bootdelay
#define CONFIG_AUTOBOOT_STOP_STR "zlg"

#define CONFIG_CFG_DEF_ADDR 		0x81000000
#define CONFIG_CFG_FILE_CNT 		0x8

#define CONFIG_ENV_SPI_BUS	0
#define CONFIG_ENV_SPI_CS	0
#define CONFIG_ENV_SPI_MAX_HZ	80000000
#define CONFIG_ENV_SPI_MODE	SPI_MODE_0

#define CONFIG_SF_DEFAULT_BUS	0
#define CONFIG_SF_DEFAULT_CS	0
#define CONFIG_SF_DEFAULT_SPEED	80000000
#define CONFIG_SF_DEFAULT_MODE	SPI_MODE_0

#define CONFIG_BIOS_NAME 	"KERNEL"
#define CONFIG_BIOSBK_NAME 	"KERBK"
#define CONFIG_ROOT_NAME 	"ROOTFS"
#define CONFIG_ROOTBK_NAME 	"ROOTBK"
#define CONFIG_APP_NAME 	"APP"
#define CONFIG_APPBK_NAME 	"APPBK"
#define CONFIG_ENV_NAME 	"ENV"
#define CONFIG_ENVBK_NAME 	"ENVBK"
#define CONFIG_DTB_NAME 	"DTB"
#define CONFIG_LOGO_NAME 	"LOGO"
#define CONFIG_UBOOT_NAME 	"UBOOT"
#define CONFIG_UBOOTBK_NAME "UB_BK"
#define CONFIG_UBOOT_ENTER 	"a_uboot_flags"


#define CONFIG_READ_ENV_A_UBOOT_SIZE 256

#define CONFIG_SPINAND_PAGE_SIZE 2048    // add for erase section size

#define CONFIG_SECTION_SIZE 4096    // add for erase section size
#define CONFIG_PARATION_TAB_SIZE 512    // add for paration table section size

#define CONFIG_SPIP_START_PAGE 0    // add for searching SPIP flag start flash offset address
#define CONFIG_SPIP_PAGE_CNT  3     // add for searching SPIP flag
#define CONFIG_PARATION_PAGE_CNT  2     // add for searching  paration table page cnt


#define CONFIG_ENV_IS_IN_SPI_FLASH 	1
#define CONFIG_ENV_SECT_SIZE 	CONFIG_ENV_SIZE

#define CONFIG_CMD_MTDPARTS  /* Enable MTD parts commands    */
#define CONFIG_MTD_PARTITIONS
#define CONFIG_MTD_DEVICE
#define CONFIG_CMD_LOAD

/* customer need detecting gpio to meet their self-defined requirement. */
#define CONFIG_ZMP_GPIO

/*
#define CONFIG_TF_UPDATE
*/

/*
 * Physical Memory Map and Memory Size 64M or128MB
 */
#define CONFIG_NR_DRAM_BANKS        		1          /* we have 1 bank of DRAM */
#define CONFIG_SYS_SDRAM_BASE       		0x80000000
#define CONFIG_SYS_SDRAM_PROTECT_SIZE       0x01000000 /* 16MB of DRAM */

/* 64MB or 128M of DRAM Config */
#ifdef CONFIG_DRAM_64M
#define PHYS_SDRAM_SIZE             		(64*1024*1024)
#else
#define PHYS_SDRAM_SIZE 					(128*1024*1024)
#endif


/*
#define CONFIG_PARTITION_TABLE_BY_CUSTOMER_ENV
*/
#define CONFIG_PARTITION_TABLE_BY_SELF_DEFINED

#ifdef  CONFIG_SPINAND_FLASH
#define  MTDIDS_DEFAULT  "nand0=spi0.0"
#define CONFIG_DEFAULT_MTDPARTS \
    "mtdparts=mtdparts=spi0.0:128K@0x0(SPL),128K@0x20000(ENV),128K@0x40000(ENVBK),"\
	"128K@0x60000(DTB),256k@0x80000(UBOOT),3M@0xC0000(KERNEL),512K@0x3c0000(LOGO),56M@0x440000(ROOTFS),-@0x3C40000(OPT)"

#define CONFIG_BOOTARGS "console=ttySAK0,115200n8 root=/dev/mtdblock7 "\
			"rootfstype=yaffs2 init=/sbin/init mem=128M memsize=128M "\
			CONFIG_DEFAULT_MTDPARTS
#else
#define  MTDIDS_DEFAULT  "nor0=spi0.0"
#define CONFIG_DEFAULT_MTDPARTS \
    "mtdparts=mtdparts=spi0.0:256K@0x0(UBOOT),64K@0x40000(ENV),64K@0x50000(ENVBK),64K@0x60000(DTB),"\
	"2560K@0x70000(KERNEL),512K@0x2f0000(LOGO),4096K@0x370000(ROOTFS),-@0x770000(WORK)"

#define CONFIG_BOOTARGS "console=ttySAK0,115200n8 root=/dev/mtdblock6 "\
			"rootfstype=squashfs init=/sbin/init mem=128M memsize=128M "\
			CONFIG_DEFAULT_MTDPARTS
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_DEFAULT_MTDPARTS"\0" \
	"a_uboot_flags=0x1\0" \
	"sf_hz=80000000\0" \
	"kernel_addr=0x70000\0" \
	"kernel_size=0x280000\0" \
	"loadaddr=0x80008000\0" \
	"dtb_addr=0x60000\0" \
	"dtb_size=0x10000\0" \
	"fdtcontroladdr=0x81300000\0" \
	"fdt_high=0xFFFFFFFF\0" \
	"console=ttySAK0,115200n8\0" \
	"mtd_root=/dev/mtdblock6\0" \
	"init=/sbin/init\0" \
	"memsize=128M\0" \
	"rootfstype=squashfs\0" \
	"setcmd=setenv bootargs console=${console} root=${mtd_root} rootfstype=${rootfstype} init=${init} mem=${memsize}\0" \
	"read_kernel=sf probe 0:0 ${sf_hz} 0; sf read ${loadaddr} ${kernel_addr} ${kernel_size}\0" \
	"read_dtb=sf probe 0:0 ${sf_hz} 0; sf read ${fdtcontroladdr} ${dtb_addr} ${dtb_size};fdt addr ${fdtcontroladdr}\0" \
	"boot_normal=run read_dtb;run read_kernel; bootm ${loadaddr} - ${fdtcontroladdr}\0" /*go ${loadaddr}*/\
	"bootcmd=run boot_normal\0" \
	"vram=12M\0" \
	"update_flag=0\0" \
	"erase_env=sf probe 0:0 ${sf_hz} 0; sf erase 0x20000 0x2000\0"/* erase env, use for test.*/ \

#define CONFIG_CMD_RUN
#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_SAVEENV


/* Define default IP addresses */
#define CONFIG_IPADDR		192.168.100.17	/* own ip address */
#define CONFIG_SERVERIP		192.168.100.5	/* used for tftp (not nfs?) */
#define CONFIG_NETMASK      255.255.255.0

/* valid baudrates */
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }
#define CONFIG_BAUDRATE		115200

#ifdef CONFIG_SPL_BOOT
/* defines for SPL */
#define CONFIG_SPL
#define CONFIG_SPL_FRAMEWORK
#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SYS_SPL_MALLOC_START	(CONFIG_SYS_TEXT_BASE - \
						CONFIG_SYS_MALLOC_LEN)
#define CONFIG_SYS_SPL_MALLOC_SIZE	CONFIG_SYS_MALLOC_LEN
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
/*#define CONFIG_SPL_LDSCRIPT	"board/$(BOARDDIR)/u-boot-spl-ipam390.lds"*/
/*#define CONFIG_SPL_STACK	0x8001ff00*/

#define CONFIG_SPL_MAX_SIZE	0x20000

/* defines for support spi */
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SPL_SPI_BUS		0
#define CONFIG_SPL_SPI_CS		0
#define CONFIG_SYS_SPI_U_BOOT_OFFS	0x20000

/* add FALCON boot mode */
#define CONFIG_CMD_SPL
#define CONFIG_SPL_OS_BOOT
#define CONFIG_SYS_NAND_SPL_KERNEL_OFFS	0x00200000
#define CONFIG_SYS_SPL_ARGS_ADDR	0x00180000
#define CONFIG_CMD_SPL_NAND_OFS		0x00180000
#define CONFIG_CMD_SPL_WRITE_SIZE	0x400
#endif

#endif	/* __CONFIG_H */
