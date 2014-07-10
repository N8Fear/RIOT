/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    board_arduino-mega2560 Arduino Mega 2560
 * @ingroup     boards
 * @brief       Board specific files for the Arduino Mega 2560 board.
 * @{
 *
 * @file        errno.h
 * @brief       Since avr-libc doesn't provide an errno header we define the
 * necessary values ourself. The numbers are taken from the errno.h from
 * the linux headers.
 *
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 */
#ifndef __ERRNO_H
#define __ERRNO_H

#define	EBUSY		16	/* Device or resource busy */
#define EINVAL      22  /* Invalid argument */
#define	EOVERFLOW	75	/* Value too large for defined data type */

#endif /** __ERRNO_H */
/** @} */
