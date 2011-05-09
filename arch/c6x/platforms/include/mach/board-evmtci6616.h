/*
 *  linux/arch/c6x/platforms/mach/board-evmtci6616.h
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
#define BOARD_RAM_SIZE	0x20000000

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

/*
 * EVM FPGA and LED definitions
 */
#define EVM_LED_OFF       0
#define EVM_LED_RED       1
#define EVM_LED_GREEN     2
#define EVM_LED_YELLOW    3

#define EVM_FPGA_LED_REG  0x02
#define EVM_FPGA_DSP_REG  0x0f

#define EVM_FPGA_LED1_S   0  /* LED 1 */
#define EVM_FPGA_LED2_S   2  /* LED 2 */
#define EVM_FPGA_LEDO_S   4  /* LED override */
#define EVM_FPGA_LED3_S   5  /* DSP LED */

#define EVM_LED_IDLE_NUM  0  /* idle is using LED1 */
#define EVM_LED_TIMER_NUM 1  /* timer is using LED2 */

