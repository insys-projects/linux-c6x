/*
 *  arch/c6x/platforms/include/mach/irq-c6670.h
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
#ifndef __MACH_IRQ_C6670_H
#define __MACH_IRQ_C6670_H

#define IRQ_EVT0        0   /* combined events */
#define IRQ_EVT1        1
#define IRQ_EVT2        2 
#define IRQ_EVT3        3
#define IRQ_TETBHFULL   4   /* TETB */
#define IRQ_TETBFULL    5
#define IRQ_TETBAXQ     6
#define IRQ_TETBOVFL    7
#define IRQ_TETBUNFL    8
#define IRQ_EMUDTDMA    9   /* emulator events */
#define IRQ_MSMC        10  /* MSMC */
#define IRQ_IDMAINT0    13  /* IDMA channel 0 interrupt */
#define IRQ_IDMAINT1    14  /* IDMA channel 1 interrupt */
#define IRQ_SEMERR      15  /* semaphores */
#define IRQ_SEM         16  /* semaphores */
#define IRQ_PCIEMSI0    17  /* PCIe */
#define IRQ_RIOINT16    20  /* RapidIO interrupts */
#define IRQ_RIOINT20    21
#define IRQ_INTC0OUT64  22  /* INTC output: 22 to 31 */
#define IRQ_QML         32  /* QM low: 32 to 47 */
#define IRQ_QMH         48  /* QM high: 48 to 55 */
#define IRQ_INTC0OUT0   56  /* INTC output */
#define IRQ_INTC0OUT1   57
#define IRQ_INTC0OUT2   58
#define IRQ_INTC0OUT3   59
#define IRQ_INTC0OUT4   60
#define IRQ_INTC0OUT5   61
#define IRQ_INTC0OUT6   62
#define IRQ_INTC0OUT7   63
#define IRQ_TINT        64  /* timer interrupts */
#define IRQ_TINTLO      64
#define IRQ_TINTHI      65
#define IRQ_TINT4       66
#define IRQ_TINTLO4     66
#define IRQ_TINTHI4     67
#define IRQ_TINT5       68
#define IRQ_TINTLO5     68
#define IRQ_TINTHI5     69
#define IRQ_TINT6       70
#define IRQ_TINTLO6     70
#define IRQ_TINTHI6     71
#define IRQ_TINT7       72
#define IRQ_TINTLO7     72
#define IRQ_TINTHI7     73
#define IRQ_INTC0OUT8   74  /* INTC output */
#define IRQ_INTC0OUT9   75
#define IRQ_INTC0OUT10  76
#define IRQ_INTC0OUT11  77
#define IRQ_GPIO_START  78  /* GPIO */
#define IRQ_GPIO4       78
#define IRQ_GPIO5       79
#define IRQ_GPIO6       80
#define IRQ_GPIO7       81
#define IRQ_GPIO8       82
#define IRQ_GPIO9       83
#define IRQ_GPIO10      84
#define IRQ_GPIO11      85
#define IRQ_GPIO12      86
#define IRQ_GPIO13      87
#define IRQ_GPIO14      88
#define IRQ_GPIO15      89
#define IRQ_IPCLOCAL    90  /* inter DSP interrupt from IPCGR */
#define IRQ_GPIOINT     91  /* local GPIO */
#define IRQ_INTC0OUT12  92
#define IRQ_INTC0OUT13  93
#define IRQ_INTC0OUT14  94
#define IRQ_INTC0OUT15  95
#define IRQ_INTERR      96  /* irq controller dropped CPU interrupt event */
#define IRQ_EMCIDMAERR  97  /* EMC invalid IDMA parameters */
#define IRQ_EFINTA      100 /* EFI interrupt from side A */
#define IRQ_EFINTB      101 /* EFI interrupt from side B */
#define IRQ_AIF0        102 /* AIF system event: 102 to 109 */
#define IRQ_MDMAERR     110 /* VbusM error event */
#define IRQ_TPCC0AET    112 /* TPCC0 AET */
#define IRQ_PMCED       113 /* single bit error detected during DMA read */
#define IRQ_TPCC1AET    114 /* TPCC1 AET */
#define IRQ_TPCC2AET    115 /* TPCC2 AET */
#define IRQ_UMCED1      116 /* corrected bit error detected */
#define IRQ_UMCED2      117 /* uncorrected bit error detected */
#define IRQ_PDCINT      118 /* PDC sleep interrupt */
#define IRQ_SYSCMPA     119 /* CPU memory protection fault */
#define IRQ_L1PCMPA     120 /* L1P CPU memory protection fault */
#define IRQ_L1PDMPA     121 /* L1P DMA memory protection fault */
#define IRQ_L1DCMPA     122 /* L1D CPU memory protection fault */
#define IRQ_L1DDMPA     123 /* L1D DMA memory protection fault */
#define IRQ_L2CMPA      124 /* L2 CPU memory protection fault */
#define IRQ_L2CDMPA     125 /* L2 DMA memory protection fault */
#define IRQ_EMCCMPA     126 /* external CPU memory protection fault */
#define IRQ_EMCBUSERR   127 /* bus error interrupt */

#define NR_SOC_IRQS	128

#endif /* __MACH_IRQ_C6670_H */
