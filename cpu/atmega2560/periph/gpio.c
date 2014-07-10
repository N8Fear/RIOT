/*
 * Copyright (C) 2014 Hauke Petersen <mail@haukepetersen.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     driver_periph
 * @{
 *
 * @file        gpio.c
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Hauke Petersen <mail@haukepetersen.de>
 *
 * @}
 */

/*
 * TODO: Implement GPIO interface
 */

#include "cpu.h"
#include "periph/gpio.h"
#include "periph_conf.h"

typedef struct {
    void (*cb)(void);
} gpio_state_t;

//static gpio_state_t config[GPIO_NUMOF];



int gpio_init_out(gpio_t dev, gpio_pp_t pushpull)
{
    return -1;
}

int gpio_init_in(gpio_t dev, gpio_pp_t pushpull)
{
    return -1;
}

int gpio_init_int(gpio_t dev, gpio_pp_t pushpull, gpio_flank_t flank, void (*cb)(void))
{
    return -1;
}

int gpio_read(gpio_t dev)
{
    return -1;
}

int gpio_set(gpio_t dev)
{
    return -1;
}

int gpio_clear(gpio_t dev)
{
    return -1;
}

int gpio_toggle(gpio_t dev)
{
    return -1;
}

int gpio_write(gpio_t dev, int value)
{
    return -1;
}
