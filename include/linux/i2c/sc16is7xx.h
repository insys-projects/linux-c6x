/*
 * include/linux/i2c/sc16is7xx.h - support NXP SC16IS7xx I2C-based UARTs
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#ifndef __LINUX_SC16IS7XX_H
#define __LINUX_SC16IS7XX_H

struct sc16is7xx_platform_data {
	unsigned	baud_base;
	unsigned	flags;
};

#define SC16IS7XX_FLAG_PRESCALE  (1 << 0)

#define PORT_16IS740 100

#endif
