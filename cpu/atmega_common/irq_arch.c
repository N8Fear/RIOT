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
 * @file        irq_arch.c
 * @brief       Implementation of the kernels irq interface
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 * @}
 */

#include <stdint.h>
#include "arch/irq_arch.h"
#include "cpu.h"

/**
 * @brief Disable all maskable interrupts
 */
unsigned int irq_arch_disable(void)
{
	/* TODO: check if 32 bit is necessary here */
    uint32_t mask = __get_interrupt_state();
    cli();
    return mask;
}

/**
 * @brief Enable all maskable interrupts
 */
unsigned int irq_arch_enable(void)
{
    sei();
    return __get_interrupt_state();
}

/**
 * @brief Restore the state of the IRQ flags
 */
void irq_arch_restore(unsigned int state)
{
	__set_interrupt_state(state);
}

/**
 * @brief See if the current context is inside an ISR
 */
int irq_arch_in(void)
{
    //TODO: find a way to implement this function
	return 0;
}
