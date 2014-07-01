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
#include <avr/io.h>

#include "board.h"
#include "cpu.h"
#include "periph/uart.h"


void led_init(void);
void SystemInit(void);
static int uart_putchar(unsigned char c, FILE *stream);
static char uart_getchar(FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
static FILE mystin = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);



void board_init(void)
{
    /* initialize stdio via USART_0 */
    SystemInit();

    /* initialize the CPU */
    cpu_init();

    /* initialize the boards LEDs */
    led_init();

    enableIRQ();
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
    DDRB = 0xFF;
    DDRH = 0xFF;
    DDRE = 0xFF;
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


/**
 * @brief Initialize the System, initialize IO via UART_0
 */
void SystemInit(void)
{
    /* initialize UART_0 for use as stdout */
    uart_init_blocking(UART_0, 38400);

    stdout = &mystdout;
    stdin = &mystin;

    /* Flush stdout */
    puts("\f");
}

static int uart_putchar(unsigned char c, FILE *stream)
{
    if (c == '\n') {
        uart_putchar('\r', stream);
    }

    uart_write_blocking(UART_0, c);
    return 0;
}

char uart_getchar(FILE *stream)
{
    char temp;
    uart_read_blocking(UART_0, &temp);
    return temp;
}
