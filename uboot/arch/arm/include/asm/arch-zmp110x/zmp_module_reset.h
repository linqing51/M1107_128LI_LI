/* 
 *	mach/reset.h
 */
#ifndef _RESET_H_
#define _RESET_H_	__FILE__

#include <asm/arch-zmp110x/zmp_cpu.h>

#define MODULE_RESET_CON1       (CHIP_CONF_BASE_ADDR + 0x20)
#define MODULE_WDT_CFG1			(CHIP_CONF_BASE_ADDR + 0xe4)
#define MODULE_WDT_CFG2			(CHIP_CONF_BASE_ADDR + 0Xe8)

#define SRESET_MMCSD		(1)
#define SRESET_SDIO		    (2)
#define SRESET_ADC			(3)
#define SRESET_DAC			(4)
#define SRESET_SPI1		    (5)
#define SRESET_SPI2		    (6)
#define SRESET_UART1		(7)
#define SRESET_UART2		(8)
#define SRESET_L2MEM		(9)
#define SRESET_I2C			(10)
#define SRESET_IRDA		    (11)
#define SRESET_GPIO		    (12)
#define SRESET_MAC			(13)
#define SRESET_ENCRY		(14)
#define SRESET_USBHS		(15)
#define SRESET_CAMERA		(19)
#define SRESET_VIDEO		(20)
#define SRESET_DRAM		    (24)

/***** extern call for comm drivers compatible *****/
#define ZMP_SRESET_MMCSD		SRESET_MMCSD
#define ZMP_SRESET_SDIO		SRESET_SDIO
#define ZMP_SRESET_ADC		    SRESET_ADC
#define ZMP_SRESET_DAC		    SRESET_DAC
#define ZMP_SRESET_SPI1		SRESET_SPI1
#define ZMP_SRESET_SPI2		SRESET_SPI2
#define ZMP_SRESET_UART1		SRESET_UART1
#define ZMP_SRESET_UART2		SRESET_UART2
#define ZMP_SRESET_L2MEM		SRESET_L2MEM
#define ZMP_SRESET_I2C		    SRESET_I2C
#define ZMP_SRESET_IRDA		SRESET_IRDA
#define ZMP_SRESET_GPIO		SRESET_GPIO
#define ZMP_SRESET_MAC		    SRESET_MAC
#define ZMP_SRESET_ENCRY		SRESET_ENCRY
#define ZMP_SRESET_USBHS		SRESET_USBHS
#define ZMP_SRESET_CAMERA	    SRESET_CAMERA
#define ZMP_SRESET_VIDEO		SRESET_VIDEO
#define ZMP_SRESET_DRAM		SRESET_DRAM


int zmp_soft_reset(u32 module);
/*** end extern call for comm drivers compatible ***/

#endif
