/*
 *  linux/arch/c6x/platforms/include/mach/edma-c6455.h
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

#ifndef _MACH_EDMA3_C6455_H
#define _MACH_EDMA3_C6455_H

/*
 * EDMA3 Channel Synchronization Events
 */
#define DMA_DSP_EVT                  0	/* HPI/PCI-to-DSP event */
#define DMA_TEVTLO0                  1	/* Timer 0 lower/high counter event */
#define DMA_TEVTHI0                  2
#define DMA_UNUSED1                  3	/* unused events from 3 to 11 */
#define DMA_MCBSP0_TX                12
#define DMA_MCBSP0_RX                13
#define DMA_MCBSP1_TX                14
#define DMA_MCBSP1_RX                15
#define DMA_TEVTLO1                  16	/* Timer 1 lower/high counter event */
#define DMA_TEVTHI1                  17
#define DMA_UNUSED2                  18	/* unused events from 18 to 19 */
#define DMA_RIOINT1                  20	/* RapidIO interrupt 1 */
#define DMA_UNUSED3                  21 /* unused events from 21 to 27 */
#define DMA_VCP_RX                   28
#define DMA_VCP_TX                   29
#define DMA_TCP_RX                   30
#define DMA_TCP_TX                   31
#define DMA_UREVT                    32	/* UTOPIA receive event */
#define DMA_UNUSED4                  33	/* unused events from 33 to 39 */
#define DMA_UXEVT                    40	/* UTOPIA transmit event */
#define DMA_UNUSED5                  41	/* unused events from 41 to 43 */
#define DMA_I2C_RX                   44
#define DMA_I2C_TX                   45
#define DMA_UNUSED6                  46	/* unused events from 46 to 47 */
#define DMA_GPIO_EVT0                48	/* GPIO events 0-15 */
#define DMA_GPIO_EVT1                49
#define DMA_GPIO_EVT2                50
#define DMA_GPIO_EVT3                51
#define DMA_GPIO_EVT4                52
#define DMA_GPIO_EVT5                53
#define DMA_GPIO_EVT6                54
#define DMA_GPIO_EVT7                55
#define DMA_GPIO_EVT8                56
#define DMA_GPIO_EVT9                57
#define DMA_GPIO_EVT10               58
#define DMA_GPIO_EVT11               59
#define DMA_GPIO_EVT12               60
#define DMA_GPIO_EVT13               61
#define DMA_GPIO_EVT14               62
#define DMA_GPIO_EVT15               63

#define MACH_EDMA_NUM_REGIONS        8
#define MACH_EDMA_NUM_QDMACH         4
#define MACH_EDMA_NUM_EVQUE	     4
#define MACH_EDMA_NUM_TC	     4

#define MACH_EDMA_IRQ_CCINT	IRQ_EDMA3CCINT0
#define MACH_EDMA_IRQ_CCERRINT	IRQ_EDMA3CCERRINT

#define IRQ_TCERRINT0	IRQ_EDMA3C0ERRINT
#define IRQ_TCERRINT1	IRQ_EDMA3C1ERRINT
#define IRQ_TCERRINT2	IRQ_EDMA3C2ERRINT
#define IRQ_TCERRINT3	IRQ_EDMA3C3ERRINT

#endif /* _MACH_EDMA3_C6455_H */
