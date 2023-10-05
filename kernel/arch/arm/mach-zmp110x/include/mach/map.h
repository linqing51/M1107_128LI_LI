/*
 * linux/arch/arm/mach-zmp110x/include/mach/map.h
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


#ifndef __ASM_ARCH_MAP_H
#define __ASM_ARCH_MAP_H

#ifndef __ASSEMBLY__
#define ZMP_ADDR(x)		((void __iomem *)0xF0000000 + (x))
#else
#define ZMP_ADDR(x)		(0xF0000000 + (x))
#endif


#define ZMP_VA_SYSCTRL		ZMP_ADDR(0x00008000)
#define ZMP_PA_SYSCTRL		(0x08000000)
#define ZMP_SZ_SYSCTRL		SZ_32K

#define ZMP_VA_CAMERA		ZMP_ADDR(0x00010000)
#define ZMP_PA_CAMERA		(0x20000000)
#define ZMP_SZ_CAMERA		SZ_64K

#define ZMP_VA_GUI			ZMP_ADDR(0x00050000)
#define ZMP_PA_GUI			(0x20040000)
#define ZMP_SZ_GUI			SZ_64K

#define ZMP_VA_SUBCTRL		ZMP_ADDR(0x00060000)
#define ZMP_PA_SUBCTRL		(0x20100000)
#define ZMP_SZ_SUBCTRL		SZ_2M

#define ZMP_VA_MIPI1			ZMP_ADDR(0x00262000)
#define ZMP_PA_MIPI1			(0x20400000)
#define ZMP_SZ_MIPI1			SZ_64K

#define ZMP_VA_MIPI2			ZMP_ADDR(0x00272000)
#define ZMP_PA_MIPI2			(0x20480000)
#define ZMP_SZ_MIPI2			SZ_64K

#define ZMP_VA_L2MEM			ZMP_ADDR(0x00294000)
#define ZMP_PA_L2MEM			(0x48000000)
#define ZMP_SZ_L2MEM			SZ_8K

#define ZMP_VA_RESERVED_MEM	ZMP_ADDR(0x00296000)
#define ZMP_PA_RESERVED_MEM	(0x81400000)
#define ZMP_SZ_RESERVED_MEM	(0x2000000)

#define ZMP_VA_L2CTRL			(ZMP_VA_SUBCTRL + 0x40000)
#define ZMP_PA_L2CTRL			(ZMP_PA_SUBCTRL + 0x40000)

#define ZMP_VA_USB			(ZMP_VA_SUBCTRL + 0x100000)
#define ZMP_PA_USB			(ZMP_PA_SUBCTRL + 0x100000)


#endif  /* __ASM_ARCH_MAP_H */

