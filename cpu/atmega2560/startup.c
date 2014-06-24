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
 * @file        startup.c
 * @brief       Startup code and interrupt vector definition
 *
 * @author     Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 * @}
 */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

/* For Catchall-Loop */
#include "board.h"
#include <util/delay.h>
#include <stdio.h>


/**
 * @brief functions for initializing the board, std-lib and kernel
 */
extern void board_init(void);
extern void kernel_init(void);
extern void __libc_init_array(void);

/**
 * @brief This pair of functions hook circumvent the call to main
 *
 * avr-libc normally uses the .init9 section for a call to main. This call
 * seems to be not replaceable without hacking inside the library. We
 * circumvent the call to main by using section .init7 to call the function
 * reset_handler which therefore is the real entry point and  section .init8
 * which should never be reached but just in case jumps to exit.
 * This way there should be no way to call main directly.
 */
void init7_ovr(void) __attribute__((naked)) __attribute__((section(".init7")));
void init8_ovr(void) __attribute__((naked)) __attribute__((section(".init8")));


void init7_ovr(void)
{
	asm("call reset_handler");
}

void init8_ovr(void)
{
	asm("jmp exit");
}
/**
 * @brief This function is the entry point after a system reset
 *
 * After a system reset, the following steps are necessary and carried out:
 * 1. load data section from flash to ram
 * 2. overwrite uninitialized data section (BSS) with zeros
 * 3. initialize the newlib
 * 4. initialize the board (sync clock, setup std-IO)
 * 5. initialize and start RIOTs kernel
 */

void reset_handler(void)
{
    /* initialize the board and startup the kernel */
    board_init();
    /* initialize std-c library (this should be done after board_init) */
/*   __libc_init_array(); // seems unnecessary for avr-libc */
    /* startup the kernel */
    kernel_init();
}

/**
 * @brief Default handler is called in case no interrupt handler was defined
 */
void dummy_handler(void)
{
    //while (1) {asm ("nop");}
	;
}

ISR(USART0_RX_vect, ISR_BLOCK)
{
	char rec_byte;
	rec_byte = UDR0;
	UDR0 = rec_byte;
}

/* interrupt handlers for manual Interrupt Vector Table
void int0_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void int1_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void int2_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void int3_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void int4_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void int5_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void int6_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void int7_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void pcint0_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void pcint1_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void pcint2_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void wdt_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer2_compa_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer2_compb_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer2_ovf_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer1_compa_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer1_compb_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer1_compc_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer1_ovf_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer0_compa_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer0_compb_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer0_ovf_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void spi_stc_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart0_rx_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart0_udre_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart0_tx_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void analog_comp_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void adc_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void ee_ready_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer3_capt_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer3_compa_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer3_compb_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer3_compc_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer3_ovf_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart1_rx_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart1_udre_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart1_tx_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void twi_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void spm_ready_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer4_capt_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer4_compa_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer4_compb_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer4_compc_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer4_ovf_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer5_capt_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer5_compa_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer5_compb_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer5_compc_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void timer5_ovf_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart2_rx_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart2_udre_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart2_tx_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart3_rx_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart3_udre_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
void usart3_tx_isr(void)                 __attribute__ ((weak, alias("dummy_handler")));
*/

/* interrupt vector table */
/* TODO: needs custom linker script
__attribute__ ((section(".vectors")))
const void *interrupt_vector[] = {
	(void*) reset_handler,			*External Pin, Power-on Reset, Brown-out, Watchdog Reset and JTAG AVR Reset */
/*	* TODO: check if this can be used to substitute main by board_init */
/*	(void*) int0_isr,				* External Interrupt Request 0 */
/*	(void*) int1_isr,				* External Interrupt Request 1 */
/*	(void*) int2_isr,				* External Interrupt Request 2 */
/*	(void*) int3_isr,				* External Interrupt Request 3 */
/*	(void*) int4_isr,				* External Interrupt Request 4 */
/*	(void*) int5_isr,				* External Interrupt Request 5 */
/*	(void*) int6_isr,				* External Interrupt Request 6 */
/*	(void*) int7_isr,				* External Interrupt Request 7 */
/*	(void*) pcint0_isr,				* Pin Change Interrupt Request 0 */
/*	(void*) pcint1_isr,				* Pin Change Interrupt Request 1 */
/*	(void*) pcint2_isr,				* Pin Change Interrupt Request 2 */
/*	(void*) wdt_isr,				* Watchdog Time-out Interrupt */
/*	(void*) timer2_compa_isr,		* Timer/Counter2 Compare Match A */
/*	(void*) timer2_compb_isr,		* Timer/Counter2 Compare Match B */
/*	(void*) timer2_ovf_isr,			* Timer/Counter2 Overflow */
/*	(void*) timer1_compa_isr,		* Timer/Counter1 Compare Match A */
/*	(void*) timer1_compb_isr,		* Timer/Counter1 Compare Match B */
/*	(void*) timer1_compc_isr,		* Timer/Counter1 Compare Match C */
/*	(void*) timer1_ovf_isr,			* Timer/Counter1 Overflow */
/*	(void*) timer0_compa_isr,		* Timer/Counter0 Compare Match A */
/*	(void*) timer0_compb_isr,		* Timer/Counter0 Compare Match B */
/*	(void*) timer0_ovf_isr,			* Timer/Counter0 Overflow */
/*	(void*) spi_stc_isr,			* SPI Serial Transfer Complete */
/*	(void*) usart0_rx_isr,			* USART0 Rx Complete */
/*	(void*) usart0_udre_isr,		* USART0 Data Register Empty */
/*	(void*) usart0_tx_isr,			* USART0 Tx Complete */
/*	(void*) analog_comp_isr,		* Analog Comparator */
/*	(void*) adc_isr,				* ADC Conversion Complete */
/*	(void*) ee_ready_isr,			* EEPROM Ready */
/*	(void*) timer3_capt_isr,		* Timer/Counter3 Capture Event */
/*	(void*) timer3_compa_isr,		* Timer/Counter3 Compare Match A */
/*	(void*) timer3_compb_isr,		* Timer/Counter3 Compare Match B */
/*	(void*) timer3_compc_isr,		* Timer/Counter3 Compare Match C */
/*	(void*) timer3_ovf_isr,			* Timer/Counter3 Overflow */
/*	(void*) usart1_rx_isr,			* USART1 Rx Complete */
/*	(void*) usart1_udre_isr,		* USART1 Data Register Empty */
/*	(void*) usart1_tx_isr,			* USART1 Tx Complete */
/*	(void*) twi_isr,				* 2-wire Serial Interface */
/*	(void*) spm_ready_isr,			* Store Program Memory Ready */
/*	(void*) timer4_capt_isr,		* Timer/Counter4 Capture Event */
/*	(void*) timer4_compa_isr,		* Timer/Counter4 Compare Match A */
/*	(void*) timer4_compb_isr,		* Timer/Counter4 Compare Match B */
/*	(void*) timer4_compc_isr,		* Timer/Counter4 Compare Match C */
/*	(void*) timer4_ovf_isr,			* Timer/Counter4 Overflow */
/*	(void*) timer5_capt_isr,		* Timer/Counter5 Capture Event */
/*	(void*) timer5_compa_isr,		* Timer/Counter5 Compare Match A */
/*	(void*) timer5_compb_isr,		* Timer/Counter5 Compare Match B */
/*	(void*) timer5_compc_isr,		* Timer/Counter5 Compare Match C */
/*	(void*) timer5_ovf_isr,			* Timer/Counter5 Overflow */
/*	(void*) usart2_rx_isr,			* USART2 Rx Complete */
/*	(void*) usart2_udre_isr,		* USART2 Data Register Empty */
/*	(void*) usart2_tx_isr,			* USART2 Tx Complete */
/*	(void*) usart3_rx_isr,			* USART3 Rx Complete */
/*	(void*) usart3_udre_isr,		* USART3 Data Register Empty */
/*	(void*) usart3_tx_isr,			* USART3 Tx Complete */
/*}; */
