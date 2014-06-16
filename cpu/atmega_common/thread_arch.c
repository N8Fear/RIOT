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

	/* AVR uses 16 Bit for pointers*/
	uint16_t *stk;
	stk = (uint16_t *)(stack_start + stack_size);

	/* put marker on stack */
	stk--;
	*stk = (uint16_t) STACK_MARKER;

	/* program counter */
	stk--;
	*stk = (uint16_t)task_func;

	/* exit */
	stk--;
	*stk = (uint16_t)sched_task_exit;

	/* status register */
	stk--;
	*stk = (uint16_t) SREG;

	/* Space for registers */
	int i;
	for (i = 0; i<32 ; i++) {
		stk--;
		*stk = i;
	}

	printf("Stack was successfully created...\n");
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

}

void thread_arch_yield(void)
{
	__context_save();

	disableIRQ();
	sched_run();
	enableIRQ();

	__context_restore();
}


__attribute__((always_inline)) static inline void __context_save(void)
{

}

__attribute__((always_inline)) static inline void __context_restore(void)
{

}
