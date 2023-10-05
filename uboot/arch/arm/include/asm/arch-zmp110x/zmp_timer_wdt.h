/* 
 *	mach/reset.h
 */
#ifndef _ZMP_TIMER_WDT_H_
#define _ZMP_TIMER_WDT_H_

#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <asm/arch-zmp110x/zmp_cpu.h>
#include <asm/arch-zmp110x/zmp_types.h> 


void wdt_enable(void);
void wdt_keepalive(unsigned int heartbeat);


#endif
