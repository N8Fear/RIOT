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
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "arch/thread_arch.h"
#include "sched.h"
#include "irq.h"
#include "cpu.h"
#include "kernel_internal.h"

/*
 * local function declarations  (prefixed with __)
 */

static void __context_save(void);
static void __context_restore(void);
static void __enter_thread_mode(void);

/**
 * @brief Since AVR doesn't support direct manipulation of the program counter we
 * model a stack like it would be left by __context_save().
 * The resulting layout in memory is the following:
 * ---------------tcb_t (not created by thread_arch_stack_init) ----------
 * local variables (a temporary value and the stackpointer)
 * -----------------------------------------------------------------------
 * a marker (AFFE) - for debugging purposes (helps finding the stack
 * -----------------------------------------------------------------------
 * a 16 Bit pointer to sched_task_exit
 * -----------------------------------------------------------------------
 * a 16 Bit pointer to task_func
 * this is placed exactly at the place where the program counter would be
 * stored normally and thus can be returned to when __context_restore()
 * has been run
 * -----------------------------------------------------------------------
 * saved registers from context:
 * r0, status register, r1 - r31
 * -----------------------------------------------------------------------
 *
 * After the invocation of __context_restore() the pointer to task_func is
 * on top of the stack and can be returned to. This way we can actually place
 * it inside of the programm counter of the MCU.
 */
char *thread_arch_stack_init(void (*task_func)(void), void *stack_start, int stack_size)
{
    uint16_t tmp_adress;
    uint8_t *stk;

    /* AVR uses 16 Bit or two 8 Bit registers for storing  pointers*/
    stk = (uint8_t *)(stack_start + stack_size);

    /* put marker on stack */
    stk--;
    *stk = (uint8_t) 0xAF;
    stk--;
    *stk = (uint8_t) 0xFE;

    /* save sched_task_exit */
    stk--;
    tmp_adress = (uint16_t) sched_task_exit;
    *stk = (uint8_t)(tmp_adress & (uint16_t) 0x00ff);
    stk--;
    tmp_adress >>= 8;
    *stk = (uint8_t)(tmp_adress & (uint16_t) 0x00ff);

	/* Garbage to align address of sched_task_exit to get popped into PC */
    stk--;
    *stk = (uint8_t) 0xEE;

    /* save address to task_func in place of the program counter */
    stk--;
    tmp_adress = (uint16_t) task_func;
    *stk = (uint8_t)(tmp_adress & (uint16_t) 0x00ff);
    stk--;
    tmp_adress >>= 8;
    *stk = (uint8_t)(tmp_adress & (uint16_t) 0x00ff);

    /* r0 */
    stk--;
    *stk = (uint8_t) 0x00;

    /* status register */
    stk--;
    *stk = (uint8_t) SREG;

    /*
     * Space for registers r1 -r31
     *
     * use loop for better readability, the compiler unrolls anyways
     */
    int i;

    for (i = 1; i <= 32 ; i++) {
        stk--;
        *stk = (uint8_t) i;
    }

    stk--;
    return (char *) stk;
}

/**
 * @brief thread_arch_stack_print prints the stack to stdout.
 * It depends on getting the correct values for stack_start, stack_size and sp
 * from sched_active_thread.
 * Maybe it would be good to change that to way that is less dependant on
 * getting correct values elsewhere (since it is a debugging tool and in the
 * presence of bugs the data may be corrupted).
 */
void thread_arch_stack_print(void)
{
    char *base_pointer = (sched_active_thread->stack_start
                          + sched_active_thread->stack_size
                          - sizeof(tcb_t) + 1
                         );
    printf("Basepointer: 0x%04X - Stacksize: %u - Stackpointer: 0x%04X\n",
           (uint16_t) base_pointer,
           sched_active_thread->stack_size,
           (uint16_t) sched_active_thread->sp);
    printf("=======Stack of %s:begin=======", sched_active_thread->name);
    uint16_t i = 0;

    while (sched_active_thread->sp != base_pointer) {
        if (i % 8 == 0) {
            printf("\n");
        }

        i++;
        printf("%02X ", (uint8_t) *base_pointer--);
    }

    printf("\n=======Stack of %s:end=======\n", sched_active_thread->name);
}

void thread_arch_start_threading(void)
{
    sched_run();
    enableIRQ();
    printf("Start Threading...\n");
    __enter_thread_mode();
}

/**
 * @brief Set the MCU into Thread-Mode and load the initial task from the stack and run it
 */
void NORETURN __enter_thread_mode(void)
{
    __context_restore();
    asm volatile("ret");

    UNREACHABLE();
}

void thread_arch_yield(void)
{
    __context_save();

    /* disableIRQ(); */ /* gets already disabled during __context_save() */
    sched_run();
    enableIRQ();

    __context_restore();
}


__attribute__((always_inline)) static inline void __context_save(void)
{
    asm volatile(
        "push r0                             \n\t"
        "in   r0, __SREG__                   \n\t"
        "cli                                 \n\t"
        "push r0                             \n\t"
        "push r1                             \n\t"
        "clr  r1                             \n\t"
        "push r2                             \n\t"
        "push r3                             \n\t"
        "push r4                             \n\t"
        "push r5                             \n\t"
        "push r6                             \n\t"
        "push r7                             \n\t"
        "push r8                             \n\t"
        "push r9                             \n\t"
        "push r10                            \n\t"
        "push r11                            \n\t"
        "push r12                            \n\t"
        "push r13                            \n\t"
        "push r14                            \n\t"
        "push r15                            \n\t"
        "push r16                            \n\t"
        "push r17                            \n\t"
        "push r18                            \n\t"
        "push r19                            \n\t"
        "push r20                            \n\t"
        "push r21                            \n\t"
        "push r22                            \n\t"
        "push r23                            \n\t"
        "push r24                            \n\t"
        "push r25                            \n\t"
        "push r26                            \n\t"
        "push r27                            \n\t"
        "push r28                            \n\t"
        "push r29                            \n\t"
        "push r30                            \n\t"
        "push r31                            \n\t"
        "lds  r26, sched_active_thread       \n\t"
        "lds  r27, sched_active_thread + 1   \n\t"
        "in   r0, __SP_L__                   \n\t"
        "st   x+, r0                         \n\t"
        "in   r0, __SP_H__                   \n\t"
        "st   x+, r0                         \n\t"
    );

}

__attribute__((always_inline)) static inline void __context_restore(void)
{
    asm volatile(
        "lds  r26, sched_active_thread       \n\t"
        "lds  r27, sched_active_thread + 1   \n\t"
        "ld   r28, x+                        \n\t"
        "out  __SP_L__, r28                  \n\t"
        "ld   r29, x+                        \n\t"
        "out  __SP_H__, r29                  \n\t"
        "pop  r31                            \n\t"
        "pop  r30                            \n\t"
        "pop  r29                            \n\t"
        "pop  r28                            \n\t"
        "pop  r27                            \n\t"
        "pop  r26                            \n\t"
        "pop  r25                            \n\t"
        "pop  r24                            \n\t"
        "pop  r23                            \n\t"
        "pop  r22                            \n\t"
        "pop  r21                            \n\t"
        "pop  r20                            \n\t"
        "pop  r19                            \n\t"
        "pop  r18                            \n\t"
        "pop  r17                            \n\t"
        "pop  r16                            \n\t"
        "pop  r15                            \n\t"
        "pop  r14                            \n\t"
        "pop  r13                            \n\t"
        "pop  r12                            \n\t"
        "pop  r11                            \n\t"
        "pop  r10                            \n\t"
        "pop  r9                             \n\t"
        "pop  r8                             \n\t"
        "pop  r7                             \n\t"
        "pop  r6                             \n\t"
        "pop  r5                             \n\t"
        "pop  r4                             \n\t"
        "pop  r3                             \n\t"
        "pop  r2                             \n\t"
        "pop  r1                             \n\t"
        "pop  r0                             \n\t"
        "out  __SREG__, r0                   \n\t"
        "pop  r0                             \n\t"
    );
}
