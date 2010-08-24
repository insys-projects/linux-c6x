/*
 *  linux/include/asm-c6x/serial.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_SERIAL_H
#define __ASM_C6X_SERIAL_H

#include <mach/board.h>

#ifndef BASE_BAUD
/*
 * This assumes you have a 1.8432 MHz clock for your UART.
 */
#define BASE_BAUD ( 1843200 / 16 )
#endif

#endif /* __ASM_C6X_SERIAL_H */
