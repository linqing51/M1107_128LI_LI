/*
 * linux/arch/arm/mach-zmp110x/include/mach/entry-macro.S
 *
 * Low-level IRQ helper macros for zmp110x platforms
 *
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


#include <mach/map.h>
#include <asm/irq.h>

#define	ZMP_IRQ_INTMASK			(ZMP_VA_SYSCTRL + 0x24)
#define	ZMP_FIQ_INTMASK			(ZMP_VA_SYSCTRL + 0x28)
#define	ZMP_INT_STATUS			(ZMP_VA_SYSCTRL + 0x4C)

	.macro  get_irqnr_preamble, base, tmp
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro	get_irqnr_and_base, irqnr, irqstat, base, tmp
	
		ldr	\base, =ZMP_INT_STATUS
		ldr	\irqstat, [\base]		@ get interrupts status
		teq	\irqstat, #0x0
		beq	1002f

		ldr	\base, =ZMP_IRQ_INTMASK	@ get interrupts mask
		ldr	\base, [\base]
		and	\irqstat, \irqstat, \base

		tst	\irqstat, #(1<<IRQ_CAMERA)
		bicne	\irqstat, \irqstat, #(1<<IRQ_CAMERA)
		movne	\irqnr, #IRQ_CAMERA
		bne	1001f

		tst	\irqstat, #(1<<IRQ_VIDEO_ENCODER)
		bicne	\irqstat, \irqstat, #(1<<IRQ_VIDEO_ENCODER)
		movne	\irqnr, #IRQ_VIDEO_ENCODER
		bne	1001f

		tst	\irqstat, #(1<<IRQ_SYSCTRL)
		bicne	\irqstat, \irqstat, #(1<<IRQ_SYSCTRL)
		movne	\irqnr, #IRQ_SYSCTRL
		bne	1001f

		tst	\irqstat, #(1<<IRQ_MCI0)
		bicne	\irqstat, \irqstat, #(1<<IRQ_MCI0)
		movne	\irqnr, #IRQ_MCI0
		bne	1001f

		tst	\irqstat, #(1<<IRQ_MCI1)
		bicne	\irqstat, \irqstat, #(1<<IRQ_MCI1)
		movne	\irqnr, #IRQ_MCI1
		bne	1001f

		tst	\irqstat, #(1<<IRQ_ADC2)
		bicne	\irqstat, \irqstat, #(1<<IRQ_ADC2)
		movne	\irqnr, #IRQ_ADC2
		bne	1001f

		tst	\irqstat, #(1<<IRQ_DAC)
		bicne	\irqstat, \irqstat, #(1<<IRQ_DAC)
		movne	\irqnr, #IRQ_DAC
		bne	1001f

		tst	\irqstat, #(1<<IRQ_SPI1)
		bicne	\irqstat, \irqstat, #(1<<IRQ_SPI1)
		movne	\irqnr, #IRQ_SPI1
		bne	1001f

		tst	\irqstat, #(1<<IRQ_SPI2)
		bicne	\irqstat, \irqstat, #(1<<IRQ_SPI2)
		movne	\irqnr, #IRQ_SPI2
		bne	1001f

		tst	\irqstat, #(1<<IRQ_UART0)
		bicne	\irqstat, \irqstat, #(1<<IRQ_UART0)
		movne	\irqnr, #IRQ_UART0
		bne	1001f

		tst	\irqstat, #(1<<IRQ_UART1)
		bicne	\irqstat, \irqstat, #(1<<IRQ_UART1)
		movne	\irqnr, #IRQ_UART1
		bne	1001f

		tst	\irqstat, #(1<<IRQ_L2MEM)
		bicne	\irqstat, \irqstat, #(1<<IRQ_L2MEM)
		movne	\irqnr, #IRQ_L2MEM
		bne	1001f

		tst	\irqstat, #(1<<IRQ_TWI0)
		bicne	\irqstat, \irqstat, #(1<<IRQ_TWI0)
		movne	\irqnr, #IRQ_TWI0
		bne	1001f

		tst	\irqstat, #(1<<IRQ_IRDA)
		bicne	\irqstat, \irqstat, #(1<<IRQ_IRDA)
		movne	\irqnr, #IRQ_IRDA
		bne	1001f

		tst	\irqstat, #(1<<IRQ_GPIO)
		bicne	\irqstat, \irqstat, #(1<<IRQ_GPIO)
		movne	\irqnr, #IRQ_GPIO
		bne	1001f

		tst	\irqstat, #(1<<IRQ_MAC)
		bicne	\irqstat, \irqstat, #(1<<IRQ_MAC)
		movne	\irqnr, #IRQ_MAC
		bne	1001f

		tst	\irqstat, #(1<<IRQ_ENCRYTION)
		bicne	\irqstat, \irqstat, #(1<<IRQ_ENCRYTION)
		movne	\irqnr, #IRQ_ENCRYTION
		bne	1001f

		tst	\irqstat, #(1<<IRQ_USBOTG_MCU)
		bicne	\irqstat, \irqstat, #(1<<IRQ_USBOTG_MCU)
		movne	\irqnr, #IRQ_USBOTG_MCU
		bne	1001f

		tst	\irqstat, #(1<<IRQ_USBOTG_DMA)
		bicne	\irqstat, \irqstat, #(1<<IRQ_USBOTG_DMA)
		movne	\irqnr, #IRQ_USBOTG_DMA
		bne	1001f

		tst	\irqstat, #(1<<IRQ_TWI1)
		bicne	\irqstat, \irqstat, #(1<<IRQ_TWI1)
		movne	\irqnr, #IRQ_TWI1
		bne	1001f
				
		tst	\irqstat, #(1<<IRQ_UART3)
		bicne	\irqstat, \irqstat, #(1<<IRQ_UART3)
		movne	\irqnr, #IRQ_UART3
		bne	1001f

		tst	\irqstat, #(1<<IRQ_UART4)
		bicne	\irqstat, \irqstat, #(1<<IRQ_UART4)
		movne	\irqnr, #IRQ_UART4
		bne	1001f

		tst	\irqstat, #(1<<IRQ_VIDEO_DECODER)
		bicne	\irqstat, \irqstat, #(1<<IRQ_VIDEO_DECODER)
		movne	\irqnr, #IRQ_VIDEO_DECODER
		bne	1001f		

		tst	\irqstat, #(1<<IRQ_GUI)
		bicne	\irqstat, \irqstat, #(1<<IRQ_GUI)
		movne	\irqnr, #IRQ_GUI
		bne	1001f

		tst	\irqstat, #(1<<IRQ_LCD)
		bicne	\irqstat, \irqstat, #(1<<IRQ_LCD)
		movne	\irqnr, #IRQ_LCD
		bne	1001f

		tst	\irqstat, #(1<<IRQ_NULL1)
		bicne	\irqstat, \irqstat, #(1<<IRQ_NULL1)
		movne	\irqnr, #IRQ_NULL1
		bne	1001f

		tst	\irqstat, #(1<<IRQ_MCI2)
		bicne	\irqstat, \irqstat, #(1<<IRQ_MCI2)
		movne	\irqnr, #IRQ_MCI2
		bne	1001f

		tst	\irqstat, #(1<<IRQ_SD_PLUGIN)
		bicne	\irqstat, \irqstat, #(1<<IRQ_SD_PLUGIN)
		movne	\irqnr, #IRQ_SD_PLUGIN
		bne	1001f

		tst	\irqstat, #(1<<IRQ_TWI2)
		bicne	\irqstat, \irqstat, #(1<<IRQ_TWI2)
		movne	\irqnr, #IRQ_TWI2
		bne	1001f

		tst	\irqstat, #(1<<IRQ_TWI3)
		bicne	\irqstat, \irqstat, #(1<<IRQ_TWI3)
		movne	\irqnr, #IRQ_TWI3
		bne	1001f

		tst	\irqstat, #(1<<IRQ_NULL2)
		bicne	\irqstat, \irqstat, #(1<<IRQ_NULL2)
		movne	\irqnr, #IRQ_NULL2
		bne	1001f
		
	1001:
		@ got irqnr
	1002:
		@ exit here
	.endm

	/* currently don't need an disable_fiq macro */
	.macro	disable_fiq
	.endm
	
