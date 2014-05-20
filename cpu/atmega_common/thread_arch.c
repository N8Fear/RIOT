/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_cortexm_common
 * @{
 *
 * @file        thread_arch.c
 * @brief       Implementation of the kernel's architecture dependent thread interface
 *
 * @author      Stefan Pfeiffer <stefan.pfeiffer@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
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
#define STACK_MARKER                (0x77777777)

/**
 * @name ARM Cortex-M specific exception return value, that triggers the return to the task mode
 *       stack pointer
 */
#define EXCEPT_RET_TASK_MODE        (0xfffffffd)


static void context_save(void);
static void context_restore(void);
static void enter_thread_mode(void);

/**
 * Cortex-M knows stacks and handles register backups, so use different stack frame layout
 *
 * TODO: How to handle different Cortex-Ms? Code is so far valid for M3 and M4 without FPU
 *
 * Layout with storage of floating point registers (applicable for Cortex-M4):
 * ------------------------------------------------------------------------------------------------------------------------------------
 * | R0 | R1 | R2 | R3 | LR | PC | xPSR | S0 | S1 | S2 | S3 | S4 | S5 | S6 | S7 | S8 | S9 | S10 | S11 | S12 | S13 | S14 | S15 | FPSCR |
 * ------------------------------------------------------------------------------------------------------------------------------------
 *
 * Layout without floating point registers:
 * --------------------------------------
 * | R0 | R1 | R2 | R3 | LR | PC | xPSR |
 * --------------------------------------
 *
 */
char *thread_arch_stack_init(void  (*task_func)(void), void *stack_start, int stack_size)
{
	char * x = (char *) &thread_arch_stack_init;
	return x;
}

void thread_arch_stack_print(void)
{
    /* TODO */
}

void thread_arch_start_threading(void)
{
	;
}

/**
 * @brief Set the MCU into Thread-Mode and load the initial task from the stack and run it
 */
void enter_thread_mode(void)
{

}

void thread_arch_yield(void)
{

}

/**
 * @brief SVC interrupt handler (to be discussed if this is really needed)
 */
__attribute__((naked)) void isr_svc(void)
{

}

__attribute__((naked)) void isr_pendsv(void)
{

}

__attribute__((always_inline)) static inline void context_save(void)
{

}

__attribute__((always_inline)) static inline void context_restore(void)
{

}
