/*
 *  arch/c6x/platforms/include/mach/irq-c6472.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __MACH_IRQ_C6472_H
#define __MACH_IRQ_C6472_H

#define IRQ_EVT0          0   /* combined events */
#define IRQ_EVT1          1
#define IRQ_EVT2          2
#define IRQ_EVT3          3
#define IRQ_RIOINT        4   /* RapidIO individual interrupt */
#define IRQ_EMACRXINT     5
#define IRQ_EMACTXINT     6
#define IRQ_EMACRXINT0    5
#define IRQ_EMACTXINT0    6
#define IRQ_EMACRXINT1    7
#define IRQ_EMACTXINT1    8
#define IRQ_EMUDTDMA      9   /* emulator events */
#define IRQ_RESERVED      10  /* reserved */
#define IRQ_EMURTDXRX     11
#define IRQ_EMURTDXTX     12
#define IRQ_IDMAINT0      13  /* IDMA channel 0 interrupt */
#define IRQ_IDMAINT1      14  /* IDMA channel 1 interrupt */
#define IRQ_EDMA3CCINT    15  /* EDMA3CC completion interrupt */
#define IRQ_TINT          16  /* timer interrupts */
#define IRQ_TINTLO        16
#define IRQ_TINTHI        17
#define IRQ_TINT6         18
#define IRQ_TINTLO6       18
#define IRQ_TINTHI6       19
#define IRQ_TINT7         20
#define IRQ_TINTLO7       20
#define IRQ_TINTHI7       21
#define IRQ_TINT8         22
#define IRQ_TINTLO8       22
#define IRQ_TINTHI8       23
#define IRQ_TINT9         24
#define IRQ_TINTLO9       24
#define IRQ_TINTHI9       25
#define IRQ_TINT10        26
#define IRQ_TINTLO10      26
#define IRQ_TINTHI10      27
#define IRQ_TINT11        28
#define IRQ_TINTLO11      28
#define IRQ_TINTHI11      29
#define IRQ_PRINT         30  /* UTOPIA-PDMA interrupts */
#define IRQ_PXINT         31
#define IRQ_RFSINT0       32  /* TSIP interrupts */
#define IRQ_RSFINT0       33
#define IRQ_XFSINT0       34
#define IRQ_XSFINT0       35
#define IRQ_RFSINT1       36
#define IRQ_RSFINT1       37
#define IRQ_XFSINT1       38
#define IRQ_XSFINT1       39
#define IRQ_RFSINT2       40
#define IRQ_RSFINT2       41
#define IRQ_XFSINT2       42
#define IRQ_XSFINT2       43
#define IRQ_RIOINT6       44  /* RapidIO common/error interrupts */
#define IRQ_RIOINT7       45
#define IRQ_ERRINT0       50  /* TSIP0 error interrupt */
#define IRQ_ERRINT1       52
#define IRQ_ERRINT2       54
#define IRQ_UINT          56  /* UTOPIA interrupt */
#define IRQ_EDMA3CCERR    57  /* EDMA3 interrupts */
#define IRQ_EDMA3CCMP     58 
#define IRQ_EDMA3TCERR0   59
#define IRQ_EDMA3TCERR1   60 
#define IRQ_EDMA3TCERR2   61 
#define IRQ_EDMA3TCERR3   62 
#define IRQ_SMCMPINT      63  /* SMC interrupts */
#define IRQ_SMCPEVT       64  /* SMC interrupts */
#define IRQ_PSCINT        65  /* PSC interrupt */
#define IRQ_EDMA3CCAETEVT 66  /* EDMA3 interrupts */
#define IRQ_EDMA3CCINT6   67
#define IRQ_EDMA3CCINT7   68
#define IRQ_EDMA3CCGINT   69
#define IRQ_EMACINT       70  /* EMAC interrupts */
#define IRQ_EMACINT0      70
#define IRQ_EMACINT1      71
#define IRQ_I2CINT        72  /* I2C interrupt */
#define IRQ_GPIOINT       73  /* GPIO interrupts */
#define IRQ_GPIO6         74
#define IRQ_GPIO7         75
#define IRQ_GPIO8         76
#define IRQ_GPIO9         77
#define IRQ_GPIO10        78
#define IRQ_GPIO11        79
#define IRQ_GPIO12        80
#define IRQ_GPIO13        81
#define IRQ_GPIO14        82
#define IRQ_GPIO15        83
#define IRQ_IPCLOCAL      84  /* inter DSP interrupt from IPCGR */
#define IRQ_HPIINT        85  /* HPI interrupt */
#define IRQ_CPUINT0       87
#define IRQ_CPUINT1       88
#define IRQ_CPUINT2       89
#define IRQ_CPUINT3       90
#define IRQ_CPUINT4       91
#define IRQ_CPUINT5       92
#define IRQ_L2PDWAKE      93  /* L2 wake interrupt */
#define IRQ_INTERR        96  /* interrupt controller dropped CPU interrupt event */
#define IRQ_EMCIDMAERR    97  /* EMC invalid IDMA parameters */
#define IRQ_PBISTINT      98  /* PBIS interrupt */
#define IRQ_EFINTA        100 /* EFI interrupt from side A */
#define IRQ_EFINTB        101 /* EFI interrupt from side B */
#define IRQ_PMCED         113 /* single bit error detected during DMA read */
#define IRQ_UMCED1        116 /* corrected bit error detected */
#define IRQ_UMCED2        117 /* uncorrected bit error detected */
#define IRQ_PDCINT        118 /* PDC sleep interrupt */
#define IRQ_L1PCMPA       120 /* L1P CPU memory protection fault */
#define IRQ_L1PDMPA       121 /* L1P DMA memory protection fault */
#define IRQ_L1DCMPA       122 /* L1D CPU memory protection fault */
#define IRQ_L1DDMPA       123 /* L1D DMA memory protection fault */
#define IRQ_L2CMPA        124 /* L2 CPU memory protection fault */
#define IRQ_L2CDMPA       125 /* L2 DMA memory protection fault */
#define IRQ_EMCCMPA       126 /* external CPU memory protection fault */
#define IRQ_EMCBUSERR     127 /* bus error interrupt */

#define NR_SOC_IRQS	  128

#endif /* __MACH_IRQ_C6472_H */
