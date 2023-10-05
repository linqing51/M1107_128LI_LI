/*
 * (C) Copyright 2010-2012
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 * Tom Rini <trini@ti.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <config.h>
#include <spl.h>
#include <image.h>
#include <linux/compiler.h>
#include <linux/mtd/mtd.h>
#include <asm/arch-zmp110x/zmp_cpu.h>

/* Pointer to as well as the global data structure for SPL */
DECLARE_GLOBAL_DATA_PTR;
gd_t gdata __attribute__ ((section(".data")));


int add_mtd_device(struct mtd_info *mtd)
{
	return 0;
}


/*
 * In the context of SPL, board_init_f must ensure that any clocks/etc for
 * DDR are enabled, ensure that the stack pointer is valid, clear the BSS
 * and call board_init_f.  We provide this version by default but mark it
 * as __weak to allow for platforms to do this in their own way if needed.
 */
void __weak board_init_f(ulong dummy)
{
	ulong value = 0;
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* Set global data pointer. */
	gd = &gdata;


		/*
	* change asic clock div to 480MHz, same as kernel for mipi&rgb lcd display.
	* same as H3D kernel.
	* set core pll channel ctrl reg bit[23], enable vclk div vld
	*/
	REG32(CORE_PLL_CHANNEL_CTRL_REG) = 0x018213F0;
	REG32(CPU_PLL_CHANNEL_CTRL_REG) |= (0x1<<28);
	while(1) {
		value = REG32(CPU_PLL_CHANNEL_CTRL_REG);
		if (!(value&(0x1<<28))) {
			break;
		}
	}


	board_init_r(NULL, 0);
}

/*
 * This function jumps to an image with argument. Normally an FDT or ATAGS
 * image.
 * arg: Pointer to paramter image in RAM
 */
#ifdef CONFIG_SPL_OS_BOOT
void __noreturn jump_to_image_linux(void *arg)
{
	unsigned long machid = 0xffffffff;
#ifdef CONFIG_MACH_TYPE
	machid = CONFIG_MACH_TYPE;
#endif

	debug("Entering kernel arg pointer: 0x%p\n", arg);
	typedef void (*image_entry_arg_t)(int, int, void *)
		__attribute__ ((noreturn));
	image_entry_arg_t image_entry =
		(image_entry_arg_t) spl_image.entry_point;
	cleanup_before_linux();
	image_entry(0, machid, arg);
}
#endif
