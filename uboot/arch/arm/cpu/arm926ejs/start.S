/*
 *  armboot - Startup Code for ARM926EJS CPU-core
 *
 *  Copyright (c) 2003  Texas Instruments
 *
 *  ----- Adapted for OMAP1610 OMAP730 from ARM925t code ------
 *
 *  Copyright (c) 2001	Marius Gröger <mag@sysgo.de>
 *  Copyright (c) 2002	Alex Züpke <azu@sysgo.de>
 *  Copyright (c) 2002	Gary Jennejohn <garyj@denx.de>
 *  Copyright (c) 2003	Richard Woodruff <r-woodruff2@ti.com>
 *  Copyright (c) 2003	Kshitij <kshitij@ti.com>
 *  Copyright (c) 2010	Albert Aribaud <albert.u.boot@aribaud.net>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm-offsets.h>
#include <config.h>
#include <common.h>
#include <version.h>
#include <asm/arch-zmp110x/zmp_cpu.h>


/*
 *************************************************************************
 *
 * Jump vector table as in table 3.1 in [1]
 *
 *************************************************************************
 */


#ifdef CONFIG_SYS_DV_NOR_BOOT_CFG
.globl _start
_start:
.globl _NOR_BOOT_CFG
_NOR_BOOT_CFG:
	.word	CONFIG_SYS_DV_NOR_BOOT_CFG
	b	reset
#elif defined(CONFIG_PLAT_ZMP)  //  step1 check ok
.globl _start
_start:
.globl _NOR_BOOT_CFG
_NOR_BOOT_CFG:
	b	reset
	.word	0x4b594e41
	.word	0x44334841
	.word	0x3E800		@download size(200Kib)
	.word	0x0		@spi devide
	.word	0x0		@resv
	.word	0x0		@single read mode
	.word	0x6		@use outsideram
#ifdef CONFIG_SPINAND_FLASH	
	.word	0x80f00000 @boot start address
#else
	.word	0x80e00000 @boot start address
#endif
	.word	0x1		@use cpu mode(default DMA mode)
    .skip   356

#ifdef CONFIG_SPINAND_FLASH	
		@	.ascii	"SPIP"
		.skip	4
#if defined(CONFIG_SPL_BUILD)
		.skip	108
		.ascii	"SP_L"
#else
		.skip	112
#endif
		.skip	1536
#else
		.ascii	"SPIP"
#if defined(CONFIG_SPL_BUILD)	
		.skip	108
		.ascii	"SP_L"
#else
		.skip	112
#endif
#endif

#ifndef CONFIG_SPINAND_FLASH
	.incbin	"H3D_SPI_NOR_InterRamBoot.dat"
#endif


	b	reset
#else
.globl _start
_start:
	b	reset
#endif
#ifdef CONFIG_SPL_BUILD
/* No exception handlers in preloader */
	ldr	pc, _hang
	ldr	pc, _hang
	ldr	pc, _hang
	ldr	pc, _hang
	ldr	pc, _hang
	ldr	pc, _hang
	ldr	pc, _hang

_hang:
	.word	do_hang
/* pad to 64 byte boundary */
	.word	0x12345678
	.word	0x12345678
	.word	0x12345678
	.word	0x12345678
	.word	0x12345678
	.word	0x12345678
	.word	0x12345678
#else
	ldr	pc, _undefined_instruction
	ldr	pc, _software_interrupt
	ldr	pc, _prefetch_abort
	ldr	pc, _data_abort
	ldr	pc, _not_used
	ldr	pc, _irq
	ldr	pc, _fiq

_undefined_instruction:
	.word undefined_instruction
_software_interrupt:
	.word software_interrupt
_prefetch_abort:
	.word prefetch_abort
_data_abort:
	.word data_abort
_not_used:
	.word not_used
_irq:
	.word irq
_fiq:
	.word fiq

#endif	/* CONFIG_SPL_BUILD */
	.balignl 16,0xdeadbeef


/*
 *************************************************************************
 *
 * Startup Code (reset vector)
 *
 * do important init only if we don't start from memory!
 * setup Memory and board specific bits prior to relocation.
 * relocate armboot to ram
 * setup stack
 *
 *************************************************************************
 */

.globl _TEXT_BASE
_TEXT_BASE:
#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_TEXT_BASE)
	.word	CONFIG_SPL_TEXT_BASE
#else
	.word	CONFIG_SYS_TEXT_BASE
#endif

/*
 * These are defined in the board-specific linker script.
 * Subtracting _start from them lets the linker put their
 * relative position in the executable instead of leaving
 * them null.
 */
.globl _bss_start_ofs
_bss_start_ofs:
	.word __bss_start - _start

.globl _bss_end_ofs
_bss_end_ofs:
	.word __bss_end - _start

.globl _end_ofs
_end_ofs:
	.word _end - _start

#ifdef CONFIG_USE_IRQ
/* IRQ stack memory (calculated at run-time) */
.globl IRQ_STACK_START
IRQ_STACK_START:
	.word	0x0badc0de

/* IRQ stack memory (calculated at run-time) */
.globl FIQ_STACK_START
FIQ_STACK_START:
	.word 0x0badc0de
#endif

/* IRQ stack memory (calculated at run-time) + 8 bytes */
.globl IRQ_STACK_START_IN
IRQ_STACK_START_IN:
	.word	0x0badc0de

/*
 * the actual reset code
 */

reset:
	/*
	 * set the cpu to SVC32 mode, step2 check ok
	 */
	mrs	r0,cpsr
	bic	r0,r0,#0x1f
	orr	r0,r0,#0xd3
	msr	cpsr,r0

	/*
	* add rgb backlight ctrl, init set gpio41 pd
	*/
#ifdef CONFIG_RGB_CORE_BOARD
	ldr r0, =0x080001a4
	ldr r1,[r0]
	orr r1,r1,#0x2
	str r1, [r0]
#endif
	/*
	 * we do sys-critical inits only at reboot,
	 * not when booting from ram!
	 */
#ifndef CONFIG_SKIP_LOWLEVEL_INIT
	bl	cpu_init_crit
#endif

	bl	_main

/*------------------------------------------------------------------------------*/

	.globl	c_runtime_cpu_setup
c_runtime_cpu_setup:

	bx	lr

/*
 *************************************************************************
 *
 * CPU_init_critical registers
 *
 * setup important registers
 * setup memory timing
 *
 *************************************************************************
 */
#ifndef CONFIG_SKIP_LOWLEVEL_INIT
cpu_init_crit:  
	/*
	 * flush D cache before disabling it, step3 check ok
	 */
	mov	r0, #0
flush_dcache:
	mrc	p15, 0, r15, c7, c10, 3
	bne	flush_dcache

	mcr	p15, 0, r0, c8, c7, 0	/* invalidate TLB */
	mcr	p15, 0, r0, c7, c5, 0	/* invalidate I Cache */

	/*
	 * disable MMU and D cache
	 * enable I cache if CONFIG_SYS_ICACHE_OFF is not defined
	 */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #0x00000300	/* clear bits 9:8 (---- --RS) */
	bic	r0, r0, #0x00000087	/* clear bits 7, 2:0 (B--- -CAM) */
#ifdef CONFIG_SYS_EXCEPTION_VECTORS_HIGH
	orr	r0, r0, #0x00002000	/* set bit 13 (--V- ----) */
#else
	bic	r0, r0, #0x00002000	/* clear bit 13 (--V- ----) */
#endif
	orr	r0, r0, #0x00000002	/* set bit 2 (A) Align */
#ifndef CONFIG_SYS_ICACHE_OFF
	orr	r0, r0, #0x00001000	/* set bit 12 (I) I-Cache */
#endif
	mcr	p15, 0, r0, c1, c0, 0

	/*
	 * Go setup Memory and board specific bits prior to relocation.
	 */
	mov	ip, lr		/* perserve link reg across call */
	bl	lowlevel_init	/* go setup pll,mux,memory */
	mov	lr, ip		/* restore link */
	mov	pc, lr		/* back to my caller */
#endif /* CONFIG_SKIP_LOWLEVEL_INIT */

#ifndef CONFIG_SPL_BUILD
/*
 *************************************************************************
 *
 * Interrupt handling
 *
 *************************************************************************
 */

@
@ IRQ stack frame.
@
#define S_FRAME_SIZE	72

#define S_OLD_R0	68
#define S_PSR		64
#define S_PC		60
#define S_LR		56
#define S_SP		52

#define S_IP		48
#define S_FP		44
#define S_R10		40
#define S_R9		36
#define S_R8		32
#define S_R7		28
#define S_R6		24
#define S_R5		20
#define S_R4		16
#define S_R3		12
#define S_R2		8
#define S_R1		4
#define S_R0		0

#define MODE_SVC 0x13
#define I_BIT	 0x80

/*
 * use bad_save_user_regs for abort/prefetch/undef/swi ...
 * use irq_save_user_regs / irq_restore_user_regs for IRQ/FIQ handling
 */

	.macro	bad_save_user_regs
	@ carve out a frame on current user stack
	sub	sp, sp, #S_FRAME_SIZE
	stmia	sp, {r0 - r12}	@ Save user registers (now in svc mode) r0-r12
	ldr	r2, IRQ_STACK_START_IN
	@ get values for "aborted" pc and cpsr (into parm regs)
	ldmia	r2, {r2 - r3}
	add	r0, sp, #S_FRAME_SIZE		@ grab pointer to old stack
	add	r5, sp, #S_SP
	mov	r1, lr
	stmia	r5, {r0 - r3}	@ save sp_SVC, lr_SVC, pc, cpsr
	mov	r0, sp		@ save current stack into r0 (param register)
	.endm

	.macro	irq_save_user_regs
	sub	sp, sp, #S_FRAME_SIZE
	stmia	sp, {r0 - r12}			@ Calling r0-r12
	@ !!!! R8 NEEDS to be saved !!!! a reserved stack spot would be good.
	add	r8, sp, #S_PC
	stmdb	r8, {sp, lr}^		@ Calling SP, LR
	str	lr, [r8, #0]		@ Save calling PC
	mrs	r6, spsr
	str	r6, [r8, #4]		@ Save CPSR
	str	r0, [r8, #8]		@ Save OLD_R0
	mov	r0, sp
	.endm

	.macro	irq_restore_user_regs
	ldmia	sp, {r0 - lr}^			@ Calling r0 - lr
	mov	r0, r0
	ldr	lr, [sp, #S_PC]			@ Get PC
	add	sp, sp, #S_FRAME_SIZE
	subs	pc, lr, #4		@ return & move spsr_svc into cpsr
	.endm

	.macro get_bad_stack
	ldr	r13, IRQ_STACK_START_IN		@ setup our mode stack

	str	lr, [r13]	@ save caller lr in position 0 of saved stack
	mrs	lr, spsr	@ get the spsr
	str	lr, [r13, #4]	@ save spsr in position 1 of saved stack
	mov	r13, #MODE_SVC	@ prepare SVC-Mode
	@ msr	spsr_c, r13
	msr	spsr, r13	@ switch modes, make sure moves will execute
	mov	lr, pc		@ capture return pc
	movs	pc, lr		@ jump to next instruction & switch modes.
	.endm

	.macro get_irq_stack			@ setup IRQ stack
	ldr	sp, IRQ_STACK_START
	.endm

	.macro get_fiq_stack			@ setup FIQ stack
	ldr	sp, FIQ_STACK_START
	.endm
#endif	/* CONFIG_SPL_BUILD */

/*
 * exception handlers,  check ok
 */
#ifdef CONFIG_SPL_BUILD
	.align	5
do_hang:
	ldr	sp, _TEXT_BASE			/* switch to abort stack */
1:
	bl	1b				/* hang and never return */
#else	/* !CONFIG_SPL_BUILD */
	.align  5
undefined_instruction:
	get_bad_stack
	bad_save_user_regs
	bl	do_undefined_instruction

	.align	5
software_interrupt:
	get_bad_stack
	bad_save_user_regs
	bl	do_software_interrupt

	.align	5
prefetch_abort:
	get_bad_stack
	bad_save_user_regs
	bl	do_prefetch_abort

	.align	5
data_abort:
	get_bad_stack
	bad_save_user_regs
	bl	do_data_abort

	.align	5
not_used:
	get_bad_stack
	bad_save_user_regs
	bl	do_not_used

#ifdef CONFIG_USE_IRQ

	.align	5
irq:
	get_irq_stack 
	irq_save_user_regs

	// load the status register of interrupt
	ldr 	r0, =INT_STATUS_REG
	ldr 	r9, [r0]
	ldr 	r0, =IRQINT_MASK_REG
	ldr 	r8, [r0]
	and 	r9, r9, r8

	mov 	r0, #0
irq_handler_check_status:					
	mov 	r1, #1
	mov 	r1, r1, lsl r0
	tst 	r9, r1
	bicne	r9, r9, r1
	stmdb	sp!, {r0}
	blne	zmp_do_irq
	ldmia	sp!, {r0}
	cmp 	r9, #0
	beq 	irq_handler_exit
	add 	r0, r0, #1
	b	  	irq_handler_check_status

irq_handler_exit:
	irq_restore_user_regs

	.align	5
fiq:
	get_fiq_stack
	/* someone ought to write a more effiction fiq_save_user_regs */
	irq_save_user_regs
	bl	do_fiq
	irq_restore_user_regs

#else

	.align	5
irq:
	get_bad_stack
	bad_save_user_regs
	bl	do_irq

	.align	5
fiq:
	get_bad_stack
	bad_save_user_regs
	bl	do_fiq

#endif
#endif	/* CONFIG_SPL_BUILD */
