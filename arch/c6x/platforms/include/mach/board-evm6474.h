/*
 *  linux/arch/c6x/platforms/mach/board-evm6474.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
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
 * I2C bus specs 
 */
#define ARCH_I2C_FREQ   400   /* KHz  */
#define ARCH_I2C_DELAY    0   /* usec */

/*
 * Timer definitions
 */
#define LINUX_TIMER_SRC (TIMER_5 - get_coreid())
#define LINUX_TIMER_EVT (IRQ_TINT5 - (get_coreid() << 1))

/*
 * Interrupt Assignments
 */
#define IRQ_IPC         INT5

/* Note. IRQ_EMAC_TX must be IRQ_EMAC_RX + 1 */
#define IRQ_EMAC_RX_0   INT6
#define IRQ_EMAC_TX_0   INT7

#define IRQ_RIO_RXTX    INT8
#define IRQ_RIO_LSU     INT9

#define IRQ_I2C	        INT13

#define IRQ_CLOCKEVENTS INT15
