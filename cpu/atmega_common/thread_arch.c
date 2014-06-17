/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_atmega_common
 * @{
 *
 * @file        thread_arch.c
 * @brief       Implementation of the kernel's architecture dependent thread interface
 *
 * @author      Stefan Pfeiffer <stefan.pfeiffer@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 * @}
 */

#include "arch/thread_arch.h"
#include "sched.h"
#include "irq.h"
#include "cpu.h"
#include "kernel_internal.h"


/**
 * @name noticeable marker marking the beginning of a stack segment
 *
 * This marker is used e.g. by *thread_arch_start_threading* to identify the stacks start.
 */
#define STACK_MARKER                (0xAFFE)


static void __context_save(void);
static void __context_restore(void);
static void enter_thread_mode(void);

char *thread_arch_stack_init(void  (*task_func)(void), void *stack_start, int stack_size)
{
	printf("Creating a new stack...\n");
	uint8_t *stk;

	/* AVR uses 16 Bit or two 8 Bit registers for storing  pointers*/
	uint16_t tmp_adress;
	stk = (uint8_t *)(stack_start + stack_size);

	/* put marker on stack */
	stk--;
	*stk = (uint8_t) 0xFE;
	stk--;
	*stk = (uint8_t) 0xAF;

	///* exit */
	//stk--;
	//*stk = (uint16_t)sched_task_exit;

	/* program counter */
	stk--;
	tmp_adress = (uint16_t) task_func;
	*stk = (uint8_t) (tmp_adress & (uint16_t) 0x00ff);
	stk--;
	tmp_adress >>= 8;
	*stk = (uint8_t) (tmp_adress & (uint16_t) 0x00ff);
	stk--;
	*stk = 0;

	/* r0 */
	stk--;
	*stk = (uint8_t) 0x00;

	/* status register */
	stk--;
	*stk = (uint8_t) SREG;

	/* EIND and RAMPZ */
	stk--;
	*stk = (uint8_t) 0x00;
	stk--;
	*stk = (uint8_t) 0x00;


	/* Space for registers r1 -r31 */
	int i;
	for (i = 0; i<31 ; i++) {
		stk--;
		*stk = i;
	}

	/* Space for saved Stackpointer */
//	stk--;
//	*stk = stk;
//
	return (char *) stk;
}

void thread_arch_stack_print(void)
{
    /* TODO */
}

void thread_arch_start_threading(void)
{
	sched_run();
	enableIRQ();
	printf("Start Threading...\n");
	enter_thread_mode();
}

/**
 * @brief Set the MCU into Thread-Mode and load the initial task from the stack and run it
 */
void NORETURN enter_thread_mode(void)
{
	PCICR  |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT0);

	/* Software interrupt */
	PINB = (1 << PB0);

	UNREACHABLE();

}

void thread_arch_yield(void)
{
	__context_save();

	disableIRQ();
	sched_run();
	enableIRQ();

	__context_restore();
}

ISR(PCINT0_vect, ISR_NAKED)
{
	sched_run();
	__context_restore();
	asm volatile ("reti");
}


__attribute__((always_inline)) static inline void __context_save(void)
{
	asm volatile (                                           \
			      "push r0                             \n\t" \
				  "in   r0, __SREG__                   \n\t" \
				  "cli                                 \n\t" \
				  "push r0                             \n\t" \
				  "in   r0, 0x3b                       \n\t" \
				  "push r0                             \n\t" \
				  "in   r0, 0x3c                       \n\t" \
				  "push r0                             \n\t" \
				  "push r1                             \n\t" \
				  "clr  r1                             \n\t" \
				  "push r2                             \n\t" \
				  "push r3                             \n\t" \
				  "push r4                             \n\t" \
				  "push r5                             \n\t" \
				  "push r6                             \n\t" \
				  "push r7                             \n\t" \
				  "push r8                             \n\t" \
				  "push r9                             \n\t" \
				  "push r10                            \n\t" \
				  "push r11                            \n\t" \
				  "push r12                            \n\t" \
				  "push r13                            \n\t" \
				  "push r14                            \n\t" \
				  "push r15                            \n\t" \
				  "push r16                            \n\t" \
				  "push r17                            \n\t" \
				  "push r18                            \n\t" \
				  "push r19                            \n\t" \
				  "push r20                            \n\t" \
				  "push r21                            \n\t" \
				  "push r22                            \n\t" \
				  "push r23                            \n\t" \
				  "push r24                            \n\t" \
				  "push r25                            \n\t" \
				  "push r26                            \n\t" \
				  "push r27                            \n\t" \
				  "push r28                            \n\t" \
				  "push r29                            \n\t" \
				  "push r30                            \n\t" \
				  "push r31                            \n\t" \
				  "lds  r26, sched_active_thread	   \n\t" \
				  "lds  r27, sched_active_thread + 1   \n\t" \
				  "in   r0, __SP_L__				   \n\t" \
				  "st   x+, r0						   \n\t" \
				  "in   r0, __SP_H__				   \n\t" \
				  "st   x+, r0						   \n\t" \
	);

}

__attribute__((always_inline)) static inline void __context_restore(void)
{
	printf("restoring: %s\n", sched_active_thread->name);
	asm volatile (                                           \
				  "lds  r26, sched_active_thread	   \n\t" \
				  "lds  r27, sched_active_thread + 1   \n\t" \
				  "ld   r28, x+						   \n\t" \
				  "out  __SP_L__, r28				   \n\t" \
				  "ld   r29, x+						   \n\t" \
				  "out  __SP_H__, r29				   \n\t" \
				  "pop  r31                            \n\t" \
				  "pop  r30                            \n\t" \
				  "pop  r29                            \n\t" \
				  "pop  r28                            \n\t" \
				  "pop  r27                            \n\t" \
				  "pop  r26                            \n\t" \
				  "pop  r25                            \n\t" \
				  "pop  r24                            \n\t" \
				  "pop  r23                            \n\t" \
				  "pop  r22                            \n\t" \
				  "pop  r21                            \n\t" \
				  "pop  r20                            \n\t" \
				  "pop  r19                            \n\t" \
				  "pop  r18                            \n\t" \
				  "pop  r17                            \n\t" \
				  "pop  r16                            \n\t" \
				  "pop  r15                            \n\t" \
				  "pop  r14                            \n\t" \
				  "pop  r13                            \n\t" \
				  "pop  r12                            \n\t" \
				  "pop  r11                            \n\t" \
				  "pop  r10                            \n\t" \
				  "pop  r9                             \n\t" \
				  "pop  r8                             \n\t" \
				  "pop  r7                             \n\t" \
				  "pop  r6                             \n\t" \
				  "pop  r5                             \n\t" \
				  "pop  r4                             \n\t" \
				  "pop  r3                             \n\t" \
				  "pop  r2                             \n\t" \
				  "pop  r1                             \n\t" \
				  "pop  r0                             \n\t" \
				  "out  0x3c, r0                       \n\t" \
				  "pop  r0                             \n\t" \
				  "out  0x3b, r0                       \n\t" \
				  "pop  r0                             \n\t" \
				  "out  __SREG__, r0                   \n\t" \
				  "pop  r0                             \n\t" \
	);
}
