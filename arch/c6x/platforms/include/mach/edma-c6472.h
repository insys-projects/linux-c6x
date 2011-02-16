/*
 *  linux/arch/c6x/platforms/include/mach/edma-c6472.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010, 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _MACH_EDMA_C6472_H
#define _MACH_EDMA_C6472_H

/* 
 * EDMA3 channels
 */
#define DMA_GPIO_EVT0		0
#define DMA_GPIO_EVT1		1
#define DMA_GPIO_EVT2		2
#define DMA_GPIO_EVT3		3
#define DMA_GPIO_EVT4		4
#define DMA_GPIO_EVT5		5
#define DMA_GPIO_EVT6		6
#define DMA_GPIO_EVT7		7
#define DMA_TINT6L		8
#define DMA_TINT6H		9
#define DMA_TINT7L		10
#define DMA_TINT7H		11
#define DMA_TINT8L		12
#define DMA_TINT8H		13
#define DMA_TINT9L		14
#define DMA_TINT9H		15
#define DMA_TINT10L		16
#define DMA_TINT10H		17
#define DMA_TINT11L		18
#define DMA_TINT11H		19
#define DMA_I2C_RX              20
#define DMA_I2C_TX              21
#define DMA_UTOPIA0_TX		22
#define DMA_UTOPIA1_TX		23
#define DMA_UTOPIA2_TX		24
#define DMA_UTOPIA3_TX		25
#define DMA_UTOPIA4_TX		26
#define DMA_UTOPIA5_TX		27
#define DMA_RIOINT4		28
#define DMA_RIOINT5		29
#define DMA_RIOINT6		30
#define DMA_RIOINT7		31
#define DMA_UNUSED              32 /* 32 - 63 unused */

/* C6472 uses region indexed on core id */
#define MACH_EDMA_NUM_REGIONS        8
#define MACH_EDMA_REGION             (get_coreid())

#define MACH_EDMA_NUM_QDMACH         4
#define MACH_EDMA_NUM_EVQUE	     4
#define MACH_EDMA_NUM_TC	     4

/*
 * This is a "local" IRQ on 6472 where regionN IRQ is
 * routed to coreN.
 */
#define MACH_EDMA_IRQ_CCINT	IRQ_EDMA3CCINT
#define MACH_EDMA_IRQ_CCERRINT	IRQ_EDMA3CCERR

#define IRQ_TCERRINT0	IRQ_EDMA3TCERR0
#define IRQ_TCERRINT1	IRQ_EDMA3TCERR1
#define IRQ_TCERRINT2	IRQ_EDMA3TCERR2
#define IRQ_TCERRINT3	IRQ_EDMA3TCERR3

#endif /* _MACH_EDMA_C6474_H */
