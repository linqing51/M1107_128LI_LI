/*
 * linux/arch/arm/mach-zmp110x/include/mach/irqs.h
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


#ifndef __ASM_ARCH_IRQS_H_
#define __ASM_ARCH_IRQS_H_

#define	ZMP_IRQ(x)				(x)
/*
 * Main CPU Interrupts
 */
#define	IRQ_MEM					ZMP_IRQ(0)
#define	IRQ_CAMERA				ZMP_IRQ(1)
#define	IRQ_VIDEO_ENCODER		ZMP_IRQ(2)
#define	IRQ_SYSCTRL				ZMP_IRQ(3)
#define	IRQ_MCI0				ZMP_IRQ(4)
#define	IRQ_MCI1				ZMP_IRQ(5)
#define	IRQ_ADC2				ZMP_IRQ(6)
#define	IRQ_DAC					ZMP_IRQ(7)
#define	IRQ_SPI1				ZMP_IRQ(8)
#define	IRQ_SPI2				ZMP_IRQ(9)
#define	IRQ_UART0				ZMP_IRQ(10)
#define	IRQ_UART1				ZMP_IRQ(11)
#define	IRQ_L2MEM				ZMP_IRQ(12)
#define	IRQ_TWI0				ZMP_IRQ(13)
#define	IRQ_IRDA				ZMP_IRQ(14)
#define	IRQ_GPIO				ZMP_IRQ(15)
#define	IRQ_MAC					ZMP_IRQ(16)
#define	IRQ_ENCRYTION			ZMP_IRQ(17)
#define	IRQ_USBOTG_MCU			ZMP_IRQ(18)
#define	IRQ_USBOTG_DMA			ZMP_IRQ(19)
#define	IRQ_TWI1				ZMP_IRQ(20)
#define	IRQ_UART3				ZMP_IRQ(21)
#define	IRQ_UART4				ZMP_IRQ(22)
#define	IRQ_VIDEO_DECODER		ZMP_IRQ(23)
#define	IRQ_GUI					ZMP_IRQ(24)
#define	IRQ_LCD					ZMP_IRQ(25)
#define	IRQ_NULL1				ZMP_IRQ(26)
#define	IRQ_MCI2				ZMP_IRQ(27)
#define	IRQ_SD_PLUGIN			ZMP_IRQ(28)
#define	IRQ_TWI2				ZMP_IRQ(29)
#define	IRQ_TWI3				ZMP_IRQ(30)
#define	IRQ_NULL2				ZMP_IRQ(31)


/*
 * System Control Module Sub-IRQs
 */
#define IRQ_SYSCTRL_START		(IRQ_NULL2 + 1)
#define	ZMP_SYSCTRL_IRQ(x)		(IRQ_SYSCTRL_START + (x))

#define IRQ_SARADC				ZMP_SYSCTRL_IRQ(0)
#define	IRQ_TIMER5				ZMP_SYSCTRL_IRQ(1)
#define	IRQ_TIMER4				ZMP_SYSCTRL_IRQ(2)
#define	IRQ_TIMER3				ZMP_SYSCTRL_IRQ(3)
#define	IRQ_TIMER2				ZMP_SYSCTRL_IRQ(4)
#define	IRQ_TIMER1				ZMP_SYSCTRL_IRQ(5)
#define IRQ_WAKEUP				ZMP_SYSCTRL_IRQ(6)
#define	IRQ_RTC_RDY				ZMP_SYSCTRL_IRQ(7)
#define	IRQ_RTC_ALARM			ZMP_SYSCTRL_IRQ(8)
#define IRQ_RTC_TIMER			ZMP_SYSCTRL_IRQ(9)
#define IRQ_RTC_WATCHDOG		ZMP_SYSCTRL_IRQ(10)
#define IRQ_GPI0_AO				ZMP_SYSCTRL_IRQ(11)
#define IRQ_GPI1_AO				ZMP_SYSCTRL_IRQ(12)
#define IRQ_GPI2_AO				ZMP_SYSCTRL_IRQ(13)
#define IRQ_GPI3_AO				ZMP_SYSCTRL_IRQ(14)
#define IRQ_GPI4_AO				ZMP_SYSCTRL_IRQ(15)
#define IRQ_BVD_LOW_PWR			ZMP_SYSCTRL_IRQ(16)
#define IRQ_PMU_RDY				ZMP_SYSCTRL_IRQ(17)
#define	IRQ_TIMER7				ZMP_SYSCTRL_IRQ(18)
#define	IRQ_TIMER6				ZMP_SYSCTRL_IRQ(19)



/* total irq number */
#define NR_IRQS     	(IRQ_TIMER6 + 123 + 1)   /* AK37D has 123 gpio irqs */

#endif  /* __ASM_ARCH_IRQS_H_ */

