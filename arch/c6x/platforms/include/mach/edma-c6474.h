/*
 *  linux/arch/c6x/platforms/include/mach/edma-c6474.h
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

#ifndef _MACH_EDMA3_C6474_H
#define _MACH_EDMA3_C6474_H

/* 
 * EDMA3 channels
 */
#define DMA_TINT0L                   0
#define DMA_TINT0H                   1
#define DMA_TINT1L                   2
#define DMA_TINT1H                   3
#define DMA_TINT2L                   4
#define DMA_TINT2H                   5
#define DMA_CIC_EVT0                 6
#define DMA_CIC_EVT1                 7
#define DMA_CIC_EVT2                 8
#define DMA_CIC_EVT3                 9
#define DMA_CIC_EVT4                 10
#define DMA_CIC_EVT5                 11
#define DMA_MCBSP0_TX                12
#define DMA_MCBSP0_RX                13
#define DMA_MCBSP1_TX                14
#define DMA_MCBSP1_RX                15
#define DMA_FSEVT4                   16
#define DMA_FSEVT5                   17
#define DMA_FSEVT6                   18
#define DMA_FSEVT7                   19
#define DMA_FSEVT8                   20
#define DMA_FSEVT9                   21
#define DMA_FSEVT10                  22
#define DMA_FSEVT11                  23
#define DMA_FSEVT12                  24
#define DMA_FSEVT13                  25
#define DMA_CIC_EVT6                 26
#define DMA_CIC_EVT7                 27
#define DMA_VCP_RX                   28
#define DMA_VCP_TX                   29
#define DMA_TCP_RX                   30
#define DMA_TCP_TX                   31
#define DMA_SEM_EVT0                 32
#define DMA_SEM_EVT1                 33
#define DMA_SEM_EVT2                 34
#define DMA_AIF_EVT0                 36
#define DMA_AIF_EVT1                 37
#define DMA_AIF_EVT2                 38
#define DMA_AIF_EVT3                 39
#define DMA_AIF_PSEVT1               40
#define DMA_AIF_PSEVT3               41
#define DMA_AIF_PSEVT5               42
#define DMA_CIC_EVT8                 43
#define DMA_I2C_RX                   44
#define DMA_I2C_TX                   45
#define DMA_CIC_EVT9                 46
#define DMA_CIC_EVT10                47
#define DMA_CIC_EVT11                48
#define DMA_CIC_EVT12                49
#define DMA_CIC_EVT13                50
#define DMA_CIC_EVT14                51
#define DMA_CIC_EVT15                52
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

/* C6474 uses region indexed on core id */
#define MACH_EDMA_NUM_REGIONS        8
#define MACH_EDMA_REGION             (7 - get_coreid())
#define IRQ_EDMA3CCINT               (IRQ_TPCCINT0 + MACH_EDMA_REGION)

#endif /* _MACH_EDMA3_C6474_H */
