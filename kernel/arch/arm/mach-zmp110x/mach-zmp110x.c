/*
 * /linux/arch/arm/mach-zmp110x/mach-zmp110x.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/irq.h>
#include <asm/sizes.h>

#include <mach/zmp_l2.h>
#include <mach/map.h>

#include <linux/input.h>
#include <linux/irqchip.h>

#define ZMP_CPU_ID			(ZMP_VA_SYSCTRL + 0x00)

#define ZMPCPU_VALUE			0x20170200
#define ZMPCPU_TYPE			"ZMP110X"

#define IODESC_ENT(x) 							\
{												\
	.virtual = (unsigned long)ZMP_VA_##x,		\
	.pfn	 = __phys_to_pfn(ZMP_PA_##x),		\
	.length	 = ZMP_SZ_##x,						\
	.type	 = MT_DEVICE						\
}

static struct map_desc zmp110x_iodesc[] __initdata = {
	IODESC_ENT(SYSCTRL),
	IODESC_ENT(CAMERA),
	IODESC_ENT(GUI),
	IODESC_ENT(SUBCTRL),
	IODESC_ENT(MIPI1),
	IODESC_ENT(MIPI2),
	IODESC_ENT(L2MEM),
	IODESC_ENT(RESERVED_MEM),
};

void __init zmp110x_map_io(void)
{
	unsigned long regval = 0x0;
    
	/* initialise the io descriptors we need for initialisation */
	iotable_init(zmp110x_iodesc, ARRAY_SIZE(zmp110x_iodesc));

	regval = __raw_readl(ZMP_CPU_ID);
	if (regval == ZMPCPU_VALUE) 
		pr_info("ZMPMCU CPU %s (ID 0x%lx)\n", ZMPCPU_TYPE, regval);
	else
		pr_info("Unknown ZMPMCU CPU ID: 0x%lx\n", regval);
		
}

void wdt_enable(void);
void wdt_keepalive(unsigned int heartbeat);

static void zmp110x_restart(enum reboot_mode mode, const char *cmd)
{
#if defined CONFIG_ZMP_WATCHDOG_TOP
	wdt_enable();
	wdt_keepalive(2);
#endif
}

static void __init zmp110x_init(void)
{	
    int ret;
	l2_init();

	ret = of_platform_populate(NULL, of_default_bus_match_table, NULL,
				   NULL);
	if (ret) {
		pr_err("of_platform_populate failed: %d\n", ret);
		BUG();
	}    
	
	return;
}


static const char * const zmp110x_dt_compat[] = {
	"zlgmcu,zmp110x",
	NULL
};

DT_MACHINE_START(ZMP110X, "ZMP110X")
	.dt_compat	= zmp110x_dt_compat,
	.map_io = zmp110x_map_io,
	.init_time = NULL,
	.init_machine = zmp110x_init,
	.init_early = NULL,
	.reserve = NULL,
    .restart = zmp110x_restart,
MACHINE_END
