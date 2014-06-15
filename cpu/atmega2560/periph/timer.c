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
#define PIN( X ) 1<<X
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
//    Tc *tim;
	/* TODO: use ticks_per_us - right now we use a static divider of 8 and
	 * therefor get one tick each 0.5 µs
	 */

    /* select the timer and enable the timer specific peripheral clocks */
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
			printf("Initializing TIMER_0...\n");
			TCCR1A = 0;
			TCCR1B = 0;

			TCNT1= 0;
			TCCR1B |= (1 << CS01);
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
			TCCR3A = 0;
			TCCR3B = 0;

			TCNT3= 0;
			TCCR3B |= (1 << CS01);
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
			TCCR4A = 0;
			TCCR4B = 0;

			TCNT4= 0;
			TCCR4B |= (1 << CS01);
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
//    Tc *tim;

	/* TODO: check how channels can be used/implemented */

    /* get timer base register address */
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
//            tim = TIMER_0_DEV;
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
//            tim = TIMER_1_DEV;
//            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
//            tim = TIMER_2_DEV;
            break;
#endif
        case TIMER_UNDEFINED:
        default:
            return -1;
    }

    /* set timeout value */
    switch (channel) {
        case 0:
//            tim->TC_CHANNEL[0].TC_RA = tim->TC_CHANNEL[0].TC_CV + timeout;
//            tim->TC_CHANNEL[0].TC_IER = TC_IER_CPAS;
            break;
        case 1:
//            tim->TC_CHANNEL[0].TC_RB = tim->TC_CHANNEL[0].TC_CV + timeout;
//            tim->TC_CHANNEL[0].TC_IER = TC_IER_CPBS;
            break;
        case 2:
//            tim->TC_CHANNEL[0].TC_RC = tim->TC_CHANNEL[0].TC_CV + timeout;
//            tim->TC_CHANNEL[0].TC_IER = TC_IER_CPCS;
            break;
        case 3:
//            tim->TC_CHANNEL[1].TC_RA = tim->TC_CHANNEL[1].TC_CV + timeout;
//            tim->TC_CHANNEL[1].TC_IER = TC_IER_CPAS;
            break;
        case 4:
//            tim->TC_CHANNEL[1].TC_RB = tim->TC_CHANNEL[1].TC_CV + timeout;
//            tim->TC_CHANNEL[1].TC_IER = TC_IER_CPBS;
            break;
        case 5:
//            tim->TC_CHANNEL[1].TC_RC = tim->TC_CHANNEL[1].TC_CV + timeout;
//            tim->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
            break;
        default:
            return -1;
    }
    return 1;
}

int timer_clear(tim_t dev, int channel)
{
//    Tc *tim;

    /* get timer base register address */
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
//            tim = TIMER_0_DEV;
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
//            tim = TIMER_1_DEV;
            break;
#endif
#if TIMER_2_EN
//        case TIMER_2:
//            tim = TIMER_2_DEV;
            break;
#endif
        case TIMER_UNDEFINED:
        default:
            return -1;
    }

    /* disable the channels interrupt */
    switch (channel) {
        case 0:
//            tim->TC_CHANNEL[0].TC_IDR = TC_IDR_CPAS;
            break;
        case 1:
//            tim->TC_CHANNEL[0].TC_IDR = TC_IDR_CPBS;
            break;
        case 2:
//            tim->TC_CHANNEL[0].TC_IDR = TC_IDR_CPCS;
            break;
        case 3:
//            tim->TC_CHANNEL[1].TC_IDR = TC_IDR_CPAS;
            break;
        case 4:
//            tim->TC_CHANNEL[1].TC_IDR = TC_IDR_CPBS;
            break;
        case 5:
//            tim->TC_CHANNEL[1].TC_IDR = TC_IDR_CPCS;
            break;
        default:
            return -1;
    }

    return 1;
}

/*
 * The timer channels 1 and 2 are configured to run with the same speed and
 * have the same value (they run in parallel), so only on of them is returned.
 */
unsigned int timer_read(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
//            return TIMER_0_DEV->TC_CHANNEL[0].TC_CV;
#endif
#if TIMER_1_EN
        case TIMER_1:
//            return TIMER_1_DEV->TC_CHANNEL[0].TC_CV;
#endif
#if TIMER_2_EN
        case TIMER_2:
//            return TIMER_2_DEV->TC_CHANNEL[0].TC_CV;
#endif
        case TIMER_UNDEFINED:
        default:
            return 0;
    }
}

/*
 * For stopping the counting of channels 1 + 2, channel 0 is disabled.
 */
void timer_stop(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
			TCCR1B &= ~((1 << CS02) | (1<< CS00));
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
			TCCR3B &= ~((1 << CS02) | (1<< CS00));
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
			TCCR4B &= ~((1 << CS02) | (1<< CS00));
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
			TCCR1B |= (1 << CS02) | (1<< CS00);
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
			TCCR3B |= (1 << CS02) | (1<< CS00);
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
			TCCR4B |= (1 << CS02) | (1<< CS00);
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
			printf("Enabling interrupt for TIMER_=...\n");
			TIMSK1 |= (1<<TOIE1);
	PORTB ^= PIN(7);
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
			TIMSK3 |= (1<<TOIE3);
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
			TIMSK4 |= (1<<TOIE4);
            break;
#endif
        case TIMER_UNDEFINED:
            break;
    }
	sei();
}

void timer_irq_disable(tim_t dev)
{
    switch (dev) {
#if TIMER_0_EN
        case TIMER_0:
			TIMSK1 &= ~(1<<TOIE1);
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
			TIMSK3 &= ~(1<<TOIE3);
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
			TIMSK3 &= ~(1<<TOIE3);
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
			TCNT1=0;
            break;
#endif
#if TIMER_1_EN
        case TIMER_1:
			TCNT3=0;
            break;
#endif
#if TIMER_2_EN
        case TIMER_2:
			TCNT4=0;
            break;
#endif
        case TIMER_UNDEFINED:
            break;
    }
}


#if TIMER_0_EN
/* TODO: implement channel logic */
ISR(TIMER1_OVF_vect, ISR_BLOCK)
{
	PORTB ^= PIN(7);
	TCNT1=0;
	//config[TIMER_0].cb(0);
}
#endif /* TIMER_0_EN */

#if TIMER_1_EN
/* TODO: implement channel logic */
ISR(TIMER3_OVF_vect)
{
	TCNT3=0;
	config[TIMER_1].cb(0);
}
#endif /* TIMER_1_EN */

#if TIMER_2_EN
/* TODO: implement channel logic */
ISR(TIMER4_OVF_vect)
{
	TCNT4=0;
	config[TIMER_2].cb(0);
	asm( "reti" );
}
#endif /* TIMER_2_EN */
