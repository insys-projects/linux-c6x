/*
 *  linux/arch/c6x/platforms/include/mach/edma.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _MACH_EDMA3_H
#define _MACH_EDMA3_H

#include <asm/hardware.h>
#include <asm/io.h>
#include <mach/board.h>

/*
 * EDMA3 base register addresses
 */
#define DMA_3PCC_BASE                0x02a00000
#define DMA_3PTC0_BASE               0x02a20000
#define DMA_3PTC1_BASE               0x02a28000
#define DMA_3PTC2_BASE               0x02a30000
#define DMA_3PTC3_BASE               0x02a38000
#define DMA_3PTC4_BASE               0x02a40000
#define DMA_3PTC5_BASE               0x02a48000

#if defined(CONFIG_SOC_TMS320C6472)
#include <mach/edma-c6472.h>
#elif defined(CONFIG_SOC_TMS320C6474)
#include <mach/edma-c6474.h>
#else
#error "No machine IRQ definitions"
#endif

#endif /* _MACH_EDMA3_H */
