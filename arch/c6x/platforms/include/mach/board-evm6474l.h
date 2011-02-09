/*
 *  linux/arch/c6x/platforms/mach/board-evm6474.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010, 2011 Texas Instruments Incorporated
 *  Author: Sandeep Paulraj <s-paulraj@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <asm/gpio.h>

/*
 * Board memory
 */
#define BOARD_RAM_SIZE	0x08000000

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
#if defined(CONFIG_DAVINCI_WATCHDOG) || defined(CONFIG_DAVINCI_WATCHDOG_MODULE)
#define LINUX_TIMER_SRC (TIMER_2 - get_coreid())
#define LINUX_TIMER_IRQ (IRQ_TINT2 - (get_coreid() << 1))
/*
 * Only timer 3-5 can reset cores.
 *    timer5 -> core0
 *    timer4 -> core1
 *    timer3 -> core2)
 */
#define LINUX_WATCHDOG_SRC (TIMER_5 - get_coreid())
#else
#define LINUX_TIMER_SRC (TIMER_5 - get_coreid())
#define LINUX_TIMER_IRQ (IRQ_TINT5 - (get_coreid() << 1))
#endif
