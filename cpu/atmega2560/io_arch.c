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
 * @file        io_arch.c
 * @brief       Implementation of the kernel's architecture dependent IO interface
 *
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>

#include "arch/io_arch.h"

int io_arch_puts(char *data, int size)
{
	//TODO: add actual implementation
	return 6;
}
