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

/* board memory map */
#define VECTADDR        0x80000000
#define TEXTADDR	0x80000400
#define TEXTLEN         0x07FFFC00
#define RAMEND		0x88000000

/*
 * Interrupt Assignments
 */
#define IRQ_CLOCKEVENTS INT15

/* Note. IRQ_EMAC_TX must be IRQ_EMAC_RX + 1 */
#define IRQ_EMAC_TX_0   INT7
#define IRQ_EMAC_RX_0   INT6


