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
 * @file        uart.c
 * @brief       Low-level UART driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author		Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 * @}
 */

#include "board.h"
#include "cpu.h"

#include "periph/uart.h"
#include "periph_conf.h"


/**
 * @brief Each UART device has to store two callbacks.
 */
typedef struct {
    void (*rx_cb)(char);
    void (*tx_cb)(void);
} uart_conf_t;

/**
 * @brief Allocate memory to store the callback functions.
 */
static uart_conf_t config[UART_NUMOF];





int uart_init(uart_t uart, uint32_t baudrate, void (*rx_cb)(char), void (*tx_cb)(void))
{
	/* initialize basic functionality */
	int res = uart_init_blocking(uart, baudrate);
	if (res != 0) {
		return res;
	}

	/* register callbacks */
	config[uart].rx_cb = rx_cb;
	config[uart].tx_cb = tx_cb;

	/* configure interrupts and enable RX interrupt */
	switch (uart) {
#if UART_0_EN
		case UART_0:
			UCSR0B |= (1 << RXCIE0);
			break;
#endif /* UART_0_EN */
#if UART_1_EN
		case UART_1:
			UCSR1B |= (1 << RXCIE1);
			break;
#endif /* UART_1_EN */
#if UART_2_EN
		case UART_2:
			UCSR2B |= (1 << RXCIE2);
			break;
#endif /* UART_2_EN */
#if UART_3_EN
		case UART_3:
			UCSR3B |= (1 << RXCIE3);
			break;
#endif /* UART_3_EN */
		case UART_UNDEFINED:
			return -2;
	}
	return 0;
}

int uart_init_blocking(uart_t uart, uint32_t baudrate)
{
	uint16_t clock_divider = F_CPU / (16 * baudrate);
	switch (uart) {
#if UART_0_EN
		case UART_0:
			/* enable RX and TX */
			UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
			/* use 8 Bit characters */
			UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);

			/* set clock divider */
			UBRR0L = clock_divider;
			UBRR0H = (clock_divider >> 8);
			break;
#endif /* UART_0 */
#if UART_1_EN
		case UART_1:
			/* enable RX and TX */
			UCSR1B |= (1 << RXEN1) | (1 << TXEN1);
			/* use 8 Bit characters */
			UCSR1C |= (1 << UCSZ10) | (1 << UCSZ11);

			/* set clock divider */
			UBRR1L = clock_divider;
			UBRR1H = (clock_divider >> 8);
			break;
#endif /* UART_1 */
#if UART_2_EN
		case UART_2:
			/* enable RX and TX */
			UCSR2B |= (1 << RXEN2) | (1 << TXEN2);
			/* use 8 Bit characters */
			UCSR2C |= (1 << UCSZ20) | (1 << UCSZ21);

			/* set clock divider */
			UBRR2L = clock_divider;
			UBRR2H = (clock_divider >> 8);
			break;
#endif /* UART_2 */
#if UART_3_EN
		case UART_3:
			/* enable RX and TX */
			UCSR3B |= (1 << RXEN3) | (1 << TXEN3);
			/* use 8 Bit characters */
			UCSR3C |= (1 << UCSZ30) | (1 << UCSZ31);

			/* set clock divider */
			UBRR3L = clock_divider;
			UBRR3H = (clock_divider >> 8);
			break;
#endif /* UART_3 */
		case UART_UNDEFINED:
			return -2;
			break;
	}
	return 0;
}

void uart_tx_begin(uart_t uart)
{

}

void uart_tx_end(uart_t uart)
{

}

int uart_write(uart_t uart, char data)
{
	switch (uart) {
#if UART_0_EN
		case UART_0:
			UDR0 = data;
			break;
#endif /* UART_0_EN */
#if UART_1_EN
		case UART_1:
			UDR1 = data;
			break;
#endif /* UART_1_EN */
#if UART_2_EN
		case UART_2:
			UDR2 = data;
			break;
#endif /* UART_2_EN */
#if UART_3_EN
		case UART_3:
			UDR3 = data;
			break;
#endif /* UART_3_EN */
		case UART_UNDEFINED:
			return -1;
	}
    return 1;
}

int uart_read_blocking(uart_t uart, char *data)
{
	switch (uart) {
#if UART_0_EN
		case UART_0:
			while (!(UCSR0A & (1 << RXC0)));
			*data = (char) UDR0;
			break;
#endif /* UART_0_EN */
#if UART_1_EN
		case UART_1:
			while (!(UCSR1A & (1 << RXC1)));
			*data = (char) UDR1;
			break;
#endif /* UART_1_EN */
#if UART_2_EN
		case UART_2:
			while (!(UCSR2A & (1 << RXC2)));
			*data = (char) UDR2;
			break;
#endif /* UART_2_EN */
#if UART_3_EN
		case UART_3:
			while (!(UCSR3A & (1 << RXC3)));
			*data = (char) UDR3;
			break;
#endif /* UART_3_EN */
		case UART_UNDEFINED:
			return -1;
	}
    return 1;
}

int uart_write_blocking(uart_t uart, char data)
{
	switch (uart) {
#if UART_0_EN
		case UART_0:
			while (!(UCSR0A & (1 << UDRE0)));
			UDR0 = data;
			break;
#endif /* UART_0_EN */
#if UART_1_EN
		case UART_1:
			while (!(UCSR1A & (1 << UDRE1)));
			UDR1 = data;
			break;
#endif /* UART_1_EN */
#if UART_2_EN
		case UART_2:
			while (!(UCSR2A & (1 << UDRE2)));
			UDR2 = data;
			break;
#endif /* UART_2_EN */
#if UART_3_EN
		case UART_3:
			while (!(UCSR3A & (1 << UDRE3)));
			UDR3 = data;
			break;
#endif /* UART_3_EN */
		case UART_UNDEFINED:
			return -1;
	}
    return 1;
}
