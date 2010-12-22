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

/*
 * Board memory 
 */
#define BOARD_RAM_SIZE	0x10000000

/* 
 * I2C UART Bridge (14.7456MHz / 16)
 */
#define BASE_BAUD	921600

/* 
 * I2C bus specs 
 */
#define ARCH_I2C_FREQ   400   /* KHz  */
#define ARCH_I2C_DELAY    0   /* usec */

/*
 * Timer definitions
 */
#define LINUX_TIMER_SRC (TIMER_0 + get_coreid())
#define LINUX_TIMER_IRQ IRQ_TINT

/*
 * Led definitions
 */
#define LED0_GPIO_PIN   12
#define LED1_GPIO_PIN   13

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
