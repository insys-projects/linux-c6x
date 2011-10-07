/*
 *  linux/arch/c6x/platforms/mach/board-scbp-tci6618.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

/* 
 * Board memory
 */
#define BOARD_RAM_SIZE	  0x40000000

/* 
 * PLL settings
 */
#define PLL_MUL           8
#define PLL_DIV2          3
#define PLL_DIV5          5
#define PLL_DIV8         64
#define PLL_OUTDIV        2

/* 
 * I2C bus specs 
 */
#define ARCH_I2C_FREQ   100   /* KHz  */
#define ARCH_I2C_DELAY    0   /* usec */

/*
 * Timer definitions
 */
#define LINUX_TIMER_SRC (TIMER_0 + get_coreid())
#define LINUX_TIMER_IRQ IRQ_TINT
