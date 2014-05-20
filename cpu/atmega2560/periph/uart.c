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
//static uart_conf_t config[UART_NUMOF];





int uart_init(uart_t uart, uint32_t baudrate, void (*rx_cb)(char), void (*tx_cb)(void))
{
    return 0;
}

int uart_init_blocking(uart_t uart, uint32_t baudrate)
{
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
    return 1;
}

int uart_read_blocking(uart_t uart, char *data)
{
    return 1;
}

int uart_write_blocking(uart_t uart, char data)
{
    return 1;
}
