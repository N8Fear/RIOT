/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_atmega2560
 * @{
 *
 * @file        hwtimer_arch.c
 * @brief       Implementation of the kernels hwtimer interface
 *
 * The hardware timer implementation uses the ATmega2560 build-in system timer as back-end.
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author		Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 * @}
 */

#include "hwtimer_arch.h"
#include "board.h"
#include "periph/timer.h"
#include "thread.h"


void irq_handler(int channel);
void (*timeout_handler)(int);


void hwtimer_arch_init(void (*handler)(int), uint32_t fcpu)
{
	;
}

void hwtimer_arch_enable_interrupt(void)
{
	;
}

void hwtimer_arch_disable_interrupt(void)
{
	;
}

void hwtimer_arch_set(unsigned long offset, short timer)
{
	;
}

void hwtimer_arch_set_absolute(unsigned long value, short timer)
{
    /* DEPRECATED?! - will not be implemented */
}

void hwtimer_arch_unset(short timer)
{
	;
}

unsigned long hwtimer_arch_now(void)
{
	return 666;
}

void irq_handler(int channel)
{
	;
}
