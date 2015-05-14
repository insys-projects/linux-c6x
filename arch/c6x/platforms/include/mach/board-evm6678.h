/*
 *  linux/arch/c6x/platforms/mach/board-evm6678.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Sandeep Paulraj <s-paulraj@ti.com>
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
#define PLL_MUL                  10
#define PLL_DIV2                 3
#define PLL_DIV5                 5
#define PLL_DIV8                 64
#define PLL_OUTDIV               2

/* 
 * I2C bus specs 
 */
#define ARCH_I2C_FREQ            100   /* KHz  */
#define ARCH_I2C_DELAY           0     /* usec */

/*
 * Timer definitions
 */
#if defined(CONFIG_DAVINCI_WATCHDOG) || defined(CONFIG_DAVINCI_WATCHDOG_MODULE)
#define LINUX_TIMER_SRC          (TIMER_8 + get_coreid())
#define LINUX_TIMER_IRQ          (IRQ_TINT8 + (get_coreid() << 1))
#define LINUX_WATCHDOG_SRC       (TIMER_0 + get_coreid())
#else
#define LINUX_TIMER_SRC          (TIMER_0 + get_coreid())
#define LINUX_TIMER_IRQ          IRQ_TINT
#endif

/*
 * EVM FPGA and LED definitions
 */
#define EVM_FPGA_MISC_REG        0x0C
#define EVM_FPGA_LED_REG         0x08

#define EVM_FPGA_RD_CMD          (1 << 7)
#define EVM_FPGA_WR_CMD          (0 << 7)

#define EVM_FPGA_MISC_NAND_WP    2
#define EVM_FPGA_MISC_XDS560     3
#define EVM_FPGA_MISC_NOR_WP     4
#define EVM_FPGA_MISC_EEPROM_WP  5
#define EVM_FPGA_MISC_PCA9306_EN 6

#define EVM_FPGA_LED1            0
#define EVM_FPGA_LED2            1
#define EVM_FPGA_LED3            2
#define EVM_FPGA_LED4            3

#define EVM_LED_ON               0
#define EVM_LED_OFF              1

#define EVM_LED_IDLE_NUM         3
#define EVM_LED_TIMER_NUM        2

/*
 * Platform halt/reset methods
 */
#define MACH_RESTART()           keystone_reset();
#define MACH_HALT()              __dint(); asm volatile(" idle");
#define MACH_POWER_OFF()         __dint(); asm volatile(" idle");
