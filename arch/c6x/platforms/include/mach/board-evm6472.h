/*
 *  linux/arch/c6x/platforms/mach/board-evm6472.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <asm/gpio.h>

/* I2C UART Bridge (14.7456MHz / 16) */
#define BASE_BAUD	921600


/* board memory map */
#define VECTADDR        0xE0000000
#define TEXTADDR	0xE0000400
#define TEXTLEN         0x0FFFFC00
#define RAMEND		0xF0000000

/* davinci i2c bus specs */
#define ARCH_DAVINCI_I2C_FREQ   400   /* KHz  */
#define ARCH_DAVINCI_I2C_DELAY    0   /* usec */

#define LED0_GPIO_PIN   GPIO_PIN12
#define LED1_GPIO_PIN   GPIO_PIN13

#ifdef CONFIG_IDLE_LED
#ifndef __ASSEMBLY__
static inline void c6x_arch_idle_led(int state)
{
	if (state)
		gpio_pin_clear(LED0_GPIO_PIN);
	else
		gpio_pin_set(LED0_GPIO_PIN);
}
#endif
#endif
