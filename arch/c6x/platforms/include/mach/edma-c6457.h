/*
 *  linux/arch/c6x/platforms/include/mach/edma-c6474.h
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

#ifndef _MACH_EDMA3_C6457_H
#define _MACH_EDMA3_C6457_H

/*
 * EDMA3 channels
 */
#define DMA_DSP_EVT		 0
#define DMA_TINT0L		 1
#define DMA_TINT0H		 2
/* 3-8 reserved */
#define DMA_ETBHFULLINT		 9
#define DMA_ETBFULLINT		10
#define DMA_ETBACQINT		11
#define DMA_MCBSP0_TX		12
#define DMA_MCBSP0_RX		13
#define DMA_MCBSP1_TX		14
#define DMA_MCBSP1_RX		15
#define DMA_TINT1L		16
#define DMA_TINT1H		17
/* 18 reserved */
#define DMA_RIOINT0		19
#define DMA_RIOINT1		20
#define DMA_RIOINT2		21
#define DMA_RIOINT3		22
#define DMA_RIOINT4		23
#define DMA_RIOINT5		24
#define DMA_RIOINT6		25
/* 26-27 reserved */
#define DMA_VCP_RX		28
#define DMA_VCP_TX		29
#define DMA_TCP2A_RX		30
#define DMA_TCP2A_TX		31
#define DMA_UTOPIA_RX		32
#define DMA_TCP2B_RX		33
#define DMA_TCP2B_TX		34
/* 35-39 reserved */
#define DMA_UTOPIA_TX		40
/* 41-43 reserved */
#define DMA_I2C_RX		44
#define DMA_I2C_TX		45
/* 46-47 reserved */
#define DMA_GPIO_EVT0		48
#define DMA_GPIO_EVT1		49
#define DMA_GPIO_EVT2		50
#define DMA_GPIO_EVT3		51
#define DMA_GPIO_EVT4		52
#define DMA_GPIO_EVT5		53
#define DMA_GPIO_EVT6		54
#define DMA_GPIO_EVT7		55
#define DMA_GPIO_EVT8		56
#define DMA_GPIO_EVT9		57
#define DMA_GPIO_EVT10		58
#define DMA_GPIO_EVT11		59
#define DMA_GPIO_EVT12		60
#define DMA_GPIO_EVT13		61
#define DMA_GPIO_EVT14		62
#define DMA_GPIO_EVT15		63

#define MACH_EDMA_NUM_REGIONS	8
#define MACH_EDMA_REGION        0
#define IRQ_EDMA3CCINT          (IRQ_TPCCINT0 + MACH_EDMA_REGION)

#define MACH_EDMA_NUM_QDMACH	8
#define MACH_EDMA_NUM_EVQUE	6
#define MACH_EDMA_NUM_TC	6

#define MACH_EDMA_IRQ_CCINT	IRQ_EDMA3CCGINT
#define MACH_EDMA_IRQ_CCERRINT	IRQ_EDMA3CCERRINT

#define IRQ_TCERRINT0	IRQ_EDMA3C0ERRINT
#define IRQ_TCERRINT1	IRQ_EDMA3C1ERRINT
#define IRQ_TCERRINT2	IRQ_EDMA3C2ERRINT
#define IRQ_TCERRINT3	IRQ_EDMA3C3ERRINT
#define IRQ_TCERRINT4	IRQ_EDMA3C4ERRINT
#define IRQ_TCERRINT5	IRQ_EDMA3C5ERRINT

#endif /* _MACH_EDMA3_C6457_H */
