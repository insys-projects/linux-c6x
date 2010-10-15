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
#define LINUX_TIMER_EVT IRQ_TINT

/*
 * Interrupt assignments
 */
#define IRQ_IPC         INT5

/* Note. IRQ_EMAC_TX must be IRQ_EMAC_RX + 1 */
#define IRQ_EMAC_RX_0   INT6
#define IRQ_EMAC_TX_0   INT7
#define IRQ_EMAC_RX_1   INT8
#define IRQ_EMAC_TX_1   INT9

#define IRQ_RIO_RXTX    INT10
#define IRQ_RIO_LSU     INT11

#define IRQ_UART_BRIDGE	INT12
#define IRQ_I2C	        INT13

#define IRQ_CLOCKEVENTS INT15

/*
 * Led definitions
 */
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
