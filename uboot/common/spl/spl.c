/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <spl.h>
#include <asm/u-boot.h>
#include <nand.h>
#include <fat.h>
#include <version.h>
#include <i2c.h>
#include <image.h>
#include <malloc.h>
#include <linux/compiler.h>
#include <spi_flash.h>


DECLARE_GLOBAL_DATA_PTR;

#ifndef CONFIG_SYS_UBOOT_START
#define CONFIG_SYS_UBOOT_START	CONFIG_SYS_TEXT_BASE
#endif
#ifndef CONFIG_SYS_MONITOR_LEN
#define CONFIG_SYS_MONITOR_LEN	(200 * 1024)
#endif

u32 *boot_params_ptr = NULL;
struct spl_image_info spl_image;

u32 part_num = 0;

/* Define board data structure */
static bd_t bdata __attribute__ ((section(".data")));

/*
 * Default function to determine if u-boot or the OS should
 * be started. This implementation always returns 1.
 *
 * Please implement your own board specific funcion to do this.
 *
 * RETURN
 * 0 to not start u-boot
 * positive if u-boot should start
 */
#ifdef CONFIG_SPL_OS_BOOT
__weak int spl_start_uboot(void)
{
	puts("SPL: Please implement spl_start_uboot() for your board\n");
	puts("SPL: Direct Linux boot not active!\n");
	return 1;
}
#endif

/*
 * Weak default function for board specific cleanup/preparation before
 * Linux boot. Some boards/platforms might not need it, so just provide
 * an empty stub here.
 */
__weak void spl_board_prepare_for_linux(void)
{
	/* Nothing to do! */
}

void spl_parse_image_header(const struct image_header *header)
{
	u32 header_size = sizeof(struct image_header);

	if (image_get_magic(header) == IH_MAGIC) {
		if (spl_image.flags & SPL_COPY_PAYLOAD_ONLY) {
			/*
			 * On some system (e.g. powerpc), the load-address and
			 * entry-point is located at address 0. We can't load
			 * to 0-0x40. So skip header in this case.
			 */
			spl_image.load_addr = image_get_load(header);
			spl_image.entry_point = image_get_ep(header);
			spl_image.size = image_get_data_size(header);
		} else {
			spl_image.entry_point = image_get_load(header);
			/* Load including the header */
			spl_image.load_addr = spl_image.entry_point -
				header_size;
			spl_image.size = image_get_data_size(header) +
				header_size;
		}
		spl_image.os = image_get_os(header);
		spl_image.name = image_get_name(header);
		debug("spl: payload image: %.*s load addr: 0x%x size: %d\n",
			sizeof(spl_image.name), spl_image.name,
			spl_image.load_addr, spl_image.size);
	} else {
		/* Signature not found - assume u-boot.bin */
		debug("mkimage signature not found - ih_magic = %x\n",
			header->ih_magic);
		/* Let's assume U-Boot will not be more than 200 KB */
		spl_image.size = CONFIG_SYS_MONITOR_LEN;
		spl_image.entry_point = CONFIG_SYS_UBOOT_START;
		spl_image.load_addr = CONFIG_SYS_TEXT_BASE;
		spl_image.os = IH_OS_U_BOOT;
		spl_image.name = "U-Boot";
	}
}

__weak void __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	typedef void __noreturn (*image_entry_noargs_t)(void);

	image_entry_noargs_t image_entry =
			(image_entry_noargs_t) spl_image->entry_point;

	debug("image entry point: 0x%X\n", spl_image->entry_point);
	image_entry();
}

#ifdef CONFIG_SPL_RAM_DEVICE
static void spl_ram_load_image(void)
{
	const struct image_header *header;

	/*
	 * Get the header.  It will point to an address defined by handoff
	 * which will tell where the image located inside the flash. For
	 * now, it will temporary fixed to address pointed by U-Boot.
	 */
	header = (struct image_header *)
		(CONFIG_SYS_TEXT_BASE -	sizeof(struct image_header));

	spl_parse_image_header(header);
}
#endif

static void bootargs_init(void)
{
	char *p1, *p2;
	char tmp[512];

	p1 = getenv("bootargs");
	p2 = getenv("mtdparts");
	memset(tmp, 0x0, 512);
	strcpy(tmp, p1);
	strcat(tmp, p2);
	strcat(tmp, " ");
	strcat(tmp, getenv("mem"));
	strcat(tmp, " ");
	strcat(tmp, getenv("memsize"));
	setenv("bootargs",tmp);
}

void board_init_r(gd_t *dummy1, ulong dummy2)
{
	u32 boot_device;
	ulong load_addr = 0;
	ulong fdtcontroladdr = 0;
	const char *p = NULL;


    unsigned int reg = 0;

//	reg = *(volatile unsigned int *)(0x08000194);
//	reg |= 1 << 21;
//	*(volatile unsigned int *)(0x08000194) = reg;

	reg = *(volatile unsigned int *)(0x0800001C);
	reg &= ~(1 << 12); 
	*(volatile unsigned int *)(0x0800001C) = reg;

	reg = *(volatile unsigned int *)(0x20170000);
	reg |= 1 << 27;
	*(volatile unsigned int *)(0x20170000) = reg;

	reg = *(volatile unsigned int *)(0x20170014);
	reg &= ~(1 << 27);
	*(volatile unsigned int *)(0x20170014) = reg;

	debug(">>spl:board_init_r()\n");

#ifdef CONFIG_SYS_SPL_MALLOC_START
	mem_malloc_init(CONFIG_SYS_SPL_MALLOC_START,
			CONFIG_SYS_SPL_MALLOC_SIZE);
#endif

#ifndef CONFIG_PPC
	/*
	 * timer_init() does not exist on PPC systems. The timer is initialized
	 * and enabled (decrementer) in interrupt_init() here.
	 */
	timer_init();
#endif

#ifdef CONFIG_SPL_BOARD_INIT
	spl_board_init();
#endif
	env_init();
	env_relocate();

	debug("efuse_read:0x%08lx\n", efuse_read());
	
	load_addr = getenv_ulong("loadaddr", 16, load_addr);
	fdtcontroladdr = getenv_ulong("fdtcontroladdr", 16, fdtcontroladdr);

	/* make at last bootargs environment */
	bootargs_init();

	/*step1: search DTB& KERNEL aera and load */
	p = getenv("mtdparts");
	part_num = spl_device_parse(p);

	boot_device = spl_boot_device();
	debug("boot device - %d\n", boot_device);
	switch (boot_device) {
#ifdef CONFIG_SPL_RAM_DEVICE
	case BOOT_DEVICE_RAM:
		spl_ram_load_image();
		break;
#endif
#ifdef CONFIG_SPL_MMC_SUPPORT
	case BOOT_DEVICE_MMC1:
	case BOOT_DEVICE_MMC2:
	case BOOT_DEVICE_MMC2_2:
		spl_mmc_load_image();
		break;
#endif
#ifdef CONFIG_SPL_NAND_SUPPORT
	case BOOT_DEVICE_NAND:
		spl_nand_load_image();
		break;
#endif
#ifdef CONFIG_SPL_ONENAND_SUPPORT
	case BOOT_DEVICE_ONENAND:
		spl_onenand_load_image();
		break;
#endif
#ifdef CONFIG_SPL_NOR_SUPPORT
	case BOOT_DEVICE_NOR:
		spl_nor_load_image();
		break;
#endif
#ifdef CONFIG_SPL_YMODEM_SUPPORT
	case BOOT_DEVICE_UART:
		spl_ymodem_load_image();
		break;
#endif
#ifdef CONFIG_SPL_SPI_SUPPORT
	case BOOT_DEVICE_SPI:
#ifndef CONFIG_SPINAND_FLASH		
		spl_spi_load_image();
#endif
		break;
	case BOOT_DEVICE_SPINAND:
#ifdef CONFIG_SPINAND_FLASH	
		spl_spinand_load_image();
#endif
		break;
#endif
#ifdef CONFIG_SPL_ETH_SUPPORT
	case BOOT_DEVICE_CPGMAC:
#ifdef CONFIG_SPL_ETH_DEVICE
		spl_net_load_image(CONFIG_SPL_ETH_DEVICE);
#else
		spl_net_load_image(NULL);
#endif
		break;
#endif
#ifdef CONFIG_SPL_USBETH_SUPPORT
	case BOOT_DEVICE_USBETH:
		spl_net_load_image("usb_ether");
		break;
#endif
	default:
		debug("SPL: Un-supported Boot Device\n");
		hang();
	}

	switch (spl_image.os) {
	case IH_OS_U_BOOT:
		printf("Jumping to U-Boot\n");
		break;
#ifdef CONFIG_SPL_OS_BOOT
	case IH_OS_LINUX:
		//printf("load kernel and dtb OK\n");
		printf("Jumping to kernel\n");
		spl_board_prepare_for_linux();
		jump_to_image_linux((void *)fdtcontroladdr);
		
#endif
	default:
		debug("Unsupported OS image.. Jumping nevertheless..\n");
	}
	jump_to_image_no_args(&spl_image);
}

/*
 * This requires UART clocks to be enabled.  In order for this to work the
 * caller must ensure that the gd pointer is valid.
 */
void preloader_console_init(void)
{
	gd->bd = &bdata;
	gd->baudrate = CONFIG_BAUDRATE;

	serial_init();		/* serial communications setup */

	gd->have_console = 1;

	puts("\nU-Boot SPL " PLAIN_VERSION " (" U_BOOT_DATE " - " \
			U_BOOT_TIME ")\n");
#ifdef CONFIG_SPL_DISPLAY_PRINT
	spl_display_print();
#endif
}
