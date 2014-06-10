/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    cpu_atmega_common ATmega common files
 * @ingroup     cpu
 * @brief       Common implementations and headers for ATmega family based micro-controllers
 * @{
 *
 * @file        cpu.h
 * @brief       Basic definitions for the ATmega common module
 *
 * When ever you want to do something hardware related, that is accessing MCUs registers directly,
 * just include this file. It will then make sure that the MCU specific headers are included.
 *
 * @author      Stefan Pfeiffer <stefan.pfeiffer@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 */

#ifndef __ATMEGA_COMMON_H
#define __ATMEGA_COMMON_H

#include "cpu-conf.h"
#include <avr/interrupt.h>

/**
 * For downwards compatibility with old RIOT code.
 * TODO: remove once core was adjusted
 */
#include "irq.h"
#define eINT            enableIRQ
#define dINT            disableIRQ

/**
 * @brief Macro returns state of the global interrupt register
 */
#define __get_interrupt_state() ({            \
        unsigned char sreg;                   \
        asm volatile ("in r0, __SREG__; \n\t" \
                      "mov %0, r0"            \
                      : "=g" (sreg)           \
					  :	                      \
		              : "r0");                \
        sreg&(1<<7);                          \
})

#define __set_interrupt_state(X) ({           \
	    asm volatile ("mov r15,%0;       \n\t"\
				      "in r16, __SREG__; \n\t"\
					  "cbr r16,7;        \n\t"\
				      "and r15,r16;      \n\t"\
				      "out __SREG__, r15 \n\t"\
				      :                       \
				      : "g" (X)               \
				      : "r15","r16");         \
})
/**
 * @brief Macro has to be called in the beginning of each ISR
 */
//#define ISR_ENTER()         asm("push {LR}")

/**
 * @brief Macro has to be called on each exit of an ISR
 */
//#define ISR_EXIT()          asm("pop {r0}"); asm("bx r0")

/**
 * @brief Initialization of the CPU
 */
void cpu_init(void);


#endif /* __ATMEGA_COMMON_H */
/** @} */
