/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     driver_periph
 * @{
 *
 * @file        timer.c
 * @brief       Low-level timer driver implementation for the ATmega2560 CPU
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 * @}
 */

#include <stdlib.h>
#include <stdio.h>

#include "board.h"
#include "cpu.h"

#include "periph/timer.h"
#include "periph_conf.h"

/* TODO: for debugging: remove me: */
#include <avr/io.h>
#include <avr/interrupt.h>

static inline int  __set_timer(tim_t dev, int channel, unsigned int timeout);

typedef struct {
    void (*cb)(int);
} timer_conf_t;

/**
 * @brief Timer state memory
 */
timer_conf_t config[TIMER_NUMOF];

/**
 * @brief Setup the given timer
 *
 * The ATmega2560 has 5 timers. Timer0 and Timer2 are 8 Bit Timers, Timer0 has
 * special uses too and therefore we'll avoid using it. Timer5 has special uses
 * with certain Arduino Shields, too. Therefore we'll also avoid using Timer5.
 *
 * Therefore we'll use the following mapping to use the left over 16 Bit
 * timers:
 *
 * Timer1 -> TIMER_0
 * Timer3 -> TIMER_1
 * Timer4 -> TIMER_2
 */
int timer_init(tim_t dev, unsigned int ticks_per_us, void (*callback)(int))
{
    /* select the timer and enable the timer specific peripheral clocks */
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            TCCR1A = 0;
            TCCR1B = 0;
            TCNT1  = 0;

            //OCR1A= ((F_CPU/10000000) ) * ticks_per_us;
            OCR1A = 1000 * ticks_per_us;

            TCCR1B |= (1 << CS02) | (1 << CS00) | (1 << WGM12);
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TCCR3A = 0;
            TCCR3B = 0;
            TCNT3  = 0;

            //OCR3A= ((F_CPU/10000000) ) * ticks_per_us;
            OCR3A = 1000 * ticks_per_us;

            TCCR3B |= (1 << CS02) | (1 << CS00) | (1 << WGM12);
            break;
#endif
#if TIMER_2_EN

        case TIMER_2:
            TCCR4A = 0;
            TCCR4B = 0;
            TCNT4  = 0;

            //OCR4A= ((F_CPU/10000000) ) * ticks_per_us;
            OCR4A = 1000 * ticks_per_us;

            TCCR4B |= (1 << CS02) | (1 << CS00) | (1 << WGM12);
            break;
#endif

        case TIMER_UNDEFINED:
        default:
            return -1;
    }

    /* save callback */
    config[dev].cb = callback;

    /* enable interrupts for given timer */
    timer_irq_enable(dev);

    return 0;
}

int timer_set(tim_t dev, int channel, unsigned int timeout)
{
    return __set_timer(dev, channel, timer_read(dev) + timeout);
}

int timer_set_absolute(tim_t dev, int channel, unsigned int timeout)
{
    return __set_timer(dev, channel, timeout);
}

int timer_clear(tim_t dev, int channel)
{
    /* get timer base register address */
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            switch (channel) {
                case 0:
                    TIFR1 &= ~(1 << OCF1A);
                    break;

                case 1:
                    TIFR1 &= ~(1 << OCF1B);
                    break;

                case 2:
                    TIFR1 &= ~(1 << OCF1C);
                    break;

                default:
                    return -1;
            }

            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            switch (channel) {
                case 0:
                    IFR3 &= ~(1 << OCF3A);
                    break;

                case 1:
                    IFR3 &= ~(1 << OCF3B);
                    break;

                case 2:
                    IFR3 &= ~(1 << OCF3C);
                    break;

                default:
                    return -1;
                    break;
            }

            break;
#endif
#if TIMER_2_EN

        case TIMER_2:
            switch (channel) {
                case 0:
                    IFR4 &= ~(1 << OCF4A);
                    break;

                case 1:
                    IFR4 &= ~(1 << OCF4B);
                    break;

                case 2:
                    IFR4 &= ~(1 << OCF4C);
                    break;

                default:
                    return -1;
                    break;
            }

            break;
#endif

        case TIMER_UNDEFINED:
        default:
            return -1;
    }

    return 1;
}

unsigned int timer_read(tim_t dev)
{
    uint16_t value;

    /*
     * Disabling interrupts globally because read from 16 Bit register can
     * otherwise be messed up
     */
    disableIRQ();

    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            value = TCNT1;
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            value = TCNT3;
            break;
#endif
#if TIMER_2_EN

        case TIMER_2:
            value = TCNT4;
            break;
#endif

        case TIMER_UNDEFINED:
        default:
            enableIRQ();
            return 0;
    }

    enableIRQ();
    return value;
}

void timer_stop(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            TCCR1B &= ~((1 << CS02) | (1 << CS00) | (1 << WGM12));
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TCCR3B &= ~((1 << CS02) | (1 << CS00) | (1 << WGM12));
            break;
#endif
#if TIMER_2_EN

        case TIMER_2:
            TCCR4B &= ~((1 << CS02) | (1 << CS00) | (1 << WGM12));
            break;
#endif

        case TIMER_UNDEFINED:
            break;
    }
}

void timer_start(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            TCCR1B |= (1 << CS02) | (1 << CS00) | (1 << WGM12);
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TCCR3B |= (1 << CS02) | (1 << CS00) | (1 << WGM12);
            break;
#endif
#if TIMER_2_EN

        case TIMER_2:
            TCCR3B |= (1 << CS02) | (1 << CS00) | (1 << WGM12);
            break;
#endif

        case TIMER_UNDEFINED:
            break;
    }
}

void timer_irq_enable(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B) | (1 << OCIE1C);
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TIMSK3 |= (1 << OCIE3A) | (1 << OCIE3B) | (1 << OCIE3C);
            break;
#endif
#if TIMER_2_EN

        case TIMER_2:
            TIMSK4 |= (1 << OCIE4A) | (1 << OCIE3B) | (1 << OCIE3C);
            break;
#endif

        case TIMER_UNDEFINED:
            break;
    }

    enableIRQ();
}

void timer_irq_disable(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            TIMSK1 &= ~(1 << OCIE1A);
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TIMSK3 &= ~(1 << OCIE3A);
            break;
#endif
#if TIMER_2_EN

        case TIMER_2:
            TIMSK4 &= ~(1 << OCIE4A);
            break;
#endif

        case TIMER_UNDEFINED:
            break;
    }
}

void timer_reset(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN

        case TIMER_0:
            TCNT1 = 0;
            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            TCNT3 = 0;
            break;
#endif
#if TIMER_2_EN

        case TIMER_2:
            TCNT4 = 0;
            break;
#endif

        case TIMER_UNDEFINED:
            break;
    }
}

inline int __set_timer(tim_t dev, int channel, unsigned int timeout)
{
    /*
     * Disabling interrupts globally because write to 16 Bit register can
     * otherwise be messed up
     */
    disableIRQ();

    switch (dev) {

#if TIMER_0_EN

        case TIMER_0:
            switch (channel) {
                case 0:
                    OCR1A = (uint16_t) timeout;
                    break;

                case 1:
                    OCR1B = (uint16_t) timeout;
                    break;

                case 2:
                    OCR1C = (uint16_t) timeout;
                    break;

                default:
                    enableIRQ();
                    return -1;
            }

            break;
#endif
#if TIMER_1_EN

        case TIMER_1:
            switch (channel) {
                case 0:
                    OCR3A = (uint16_t) timeout;
                    break;

                case 1:
                    OCR3B = (uint16_t) timeout;
                    break;

                case 2:
                    OCR3C = (uint16_t) timeout;
                    break;

                default:
                    enableIRQ();
                    return -1;
            }

            break;
#endif
#if TIMER_2_EN

        case TIMER_2:
            switch (channel) {
                case 0:
                    OCR4A = (uint16_t) timeout;
                    break;

                case 1:
                    OCR4B = (uint16_t) timeout;
                    break;

                case 2:
                    OCR4C = (uint16_t) timeout;
                    break;

                default:
                    enableIRQ();
                    return -1;
            }

            break;
#endif

        case TIMER_UNDEFINED:
        default:
            enableIRQ();
            return -1;
    }

    enableIRQ();
    return 1;
}
#if TIMER_0_EN
ISR(TIMER1_COMPA_vect, ISR_BLOCK)
{
    PORTB ^= (1 << 7); // for debugging: shows that the ISR is used
    //config[TIMER_0].cb(0);
}
ISR(TIMER1_COMPB_vect, ISR_BLOCK)
{
    //config[TIMER_0].cb(1);
}
ISR(TIMER1_COMPC_vect, ISR_BLOCK)
{
    //config[TIMER_0].cb(2);
}
#endif /* TIMER_0_EN */

#if TIMER_1_EN
ISR(TIMER3_COMPA_vect, ISR_BLOCK)
{
    //config[TIMER_1].cb(0);
}
ISR(TIMER3_COMPB_vect, ISR_BLOCK)
{
    //config[TIMER_1].cb(1);
}
ISR(TIMER3_COMPC_vect, ISR_BLOCK)
{
    //config[TIMER_1].cb(2);
}
#endif /* TIMER_1_EN */

#if TIMER_2_EN
ISR(TIMER4_COMPA_vect, ISR_BLOCK)
{
    //config[TIMER_2].cb(0);
}
ISR(TIMER5_COMPB_vect, ISR_BLOCK)
{
    //config[TIMER_2].cb(1);
}
ISR(TIMER6_COMPC_vect, ISR_BLOCK)
{
    //config[TIMER_2].cb(2);
}
#endif /* TIMER_2_EN */
