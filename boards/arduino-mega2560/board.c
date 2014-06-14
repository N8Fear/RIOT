/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_arduino-mega2560
 * @{
 *
 * @file        board.c
 * @brief       Board specific implementations for the Arduino Mega 2560 board
 *
 * @author	    Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "board.h"
#include "cpu.h"

/* defines from extern project */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define USART_BAUDRATE 9600
#define BAUD_PRESCALE ((F_CPU / (USART_BAUDRATE * 16UL)) - 1)

void init_uart(void)
{
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0); // turn on rx and tx
	UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); // 8 Bit characters

	UBRR0H = (BAUD_PRESCALE >> 8);
	UBRR0L = BAUD_PRESCALE;

	UCSR0B |= (1 << RXCIE0);
}

static int uart_putchar(unsigned char c, FILE *stream)
{
	if (c == '\n')
			uart_putchar('\r', stream);
	while ( !(UCSR0A & (1 << UDRE0)) )
		;
	UDR0 = c;
	return 0;
}

void led_init(void);

/* prototypes from extern project */
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

/*prototypes: end */

void board_init(void)
{
    /* initialize core clocks via CMSIS function provided by Atmel */
//    SystemInit();

    /* initialize the CPU */
    cpu_init();

    /* initialize the boards LEDs */
    led_init();

	/* hardcoded UART stuff from extern project */
	init_uart();
	/* Global interrupts enable */
	sei();

	stdout=&mystdout;
	/* Flush stdout */
	puts("\r");
}


/**
 * @brief Initialize the boards on-board LED (Amber LED "L")
 *
 * The LED initialization is hard-coded in this function. As the LED is soldered
 * onto the board it is fixed to its CPU pins.
 *
 * The LED is connected to the following pin:
 * - LED: PB27
 */
void led_init(void)
{
//    /* enable PIO control of pin PD27 */
//    LED_PORT->PIO_PER = LED_PIN;
//    /* set pin as output */
//    LED_PORT->PIO_OER = LED_PIN;
//    /* enable direct write access to the LED pin */
//    LED_PORT->PIO_OWER = LED_PIN;
//    /* disable pull-up */
//    LED_PORT->PIO_PUDR = LED_PIN;
//    /* clear pin */
//    LED_PORT->PIO_CODR = LED_PIN;
}
