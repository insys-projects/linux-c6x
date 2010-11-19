/*
 *  arch/c6x/platforms/include/mach/irq-c645x.h
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
#ifndef __MACH_IRQ_C645X_H
#define __MACH_IRQ_C645X_H

/*
 * C645x peripheral interrupt sources
 */
#define IRQ_EVT0          0   /* combined events */
#define IRQ_EVT1          1
#define IRQ_EVT2          2
#define IRQ_EVT3          3
#define IRQ_EMUDTDMA      9   /* emulator events */
#define IRQ_EMURTDXRX     11
#define IRQ_EMURTDXTX     12
#define IRQ_IDMAINT0      13  /* IDMA channel 0 event */
#define IRQ_IDMAINT1      14  /* IDMA channel 1 event */
#define IRQ_DSPINT        15  /* HPI/PCI-to-DSP event */
#define IRQ_I2CINT        16  /* I2C event */
#define IRQ_EMACINT       17  /* EMAC event */
#define IRQ_AEASYNCERR    18  /* EMIFA error event */
#define IRQ_RIOINT0       20  /* RapidIO events */
#define IRQ_RIOINT1       21
#if defined(CONFIG_SOC_TMS320C6455)
#define IRQ_RIOINT4       22
#endif
#if defined(CONFIG_SOC_TMS320C6457)
#define IRQ_RIOINT2       23
#define IRQ_RIOINT3       24
#endif
#define IRQ_EDMA3CCGINT   24  /* EDMA channel completion event */
#if defined(CONFIG_SOC_TMS320C6457)
#define IRQ_EMACRXINT     25
#define IRQ_EMACTXINT     26
#define IRQ_EMACTHRES     27
#define IRQ_RIOINT4       28
#define IRQ_RIOINT5       28
#define IRQ_RIOINT6       28
#else
#define IRQ_EMACRXINT     IRQ_EMACINT
#define IRQ_EMACTXINT     IRQ_EMACINT
#endif
#define IRQ_VCP2INT       32  /* VCP2 error event */
#if defined(CONFIG_SOC_TMS320C6455)
#define IRQ_TCP2INT       33  /* TCP2 error event */
#endif
#if defined(CONFIG_SOC_TMS320C6457)
#define IRQ_TCP2AINT      33  /* TCP2 error event */
#define IRQ_TCP2BINT      34  /* TCP2 error event */
#endif
#define IRQ_UINT          36  /* UTOPIA event */
#define IRQ_RINT0         40  /* McBSP event */
#define IRQ_XINT0         41
#define IRQ_RINT1         42
#define IRQ_XINT1         43
#define IRQ_GPIO0         51  /* GPIO events */
#define IRQ_GPIO1         52
#define IRQ_GPIO2         53
#define IRQ_GPIO3         54
#define IRQ_GPIO4         55
#define IRQ_GPIO5         56
#define IRQ_GPIO6         57
#define IRQ_GPIO7         58
#define IRQ_GPIO8         59
#define IRQ_GPIO9         60
#define IRQ_GPIO10        61
#define IRQ_GPIO11        62
#define IRQ_GPIO12        63
#define IRQ_GPIO13        64
#define IRQ_GPIO14        65
#define IRQ_GPIO15        66
#define IRQ_TINT0         67  /* timer events */
#define IRQ_TINTLO0       67
#define IRQ_TINTHI0       68
#define IRQ_TINT1         69
#define IRQ_TINTLO1       69
#define IRQ_TINTHI1       70
#define IRQ_EDMA3CCINT0   71  /* EDMA events */
#define IRQ_EDMA3CCINT1   72
#define IRQ_EDMA3CCINT2   73
#define IRQ_EDMA3CCINT3   74
#define IRQ_EDMA3CCINT4   75
#define IRQ_EDMA3CCINT5   76
#define IRQ_EDMA3CCINT6   77
#define IRQ_EDMA3CCINT7   78
#define IRQ_EDMA3CCERRINT 79
#define IRQ_EDMA3CCMPINT  80
#define IRQ_EDMA3C0ERRINT 81
#define IRQ_EDMA3C1ERRINT 82
#define IRQ_EDMA3C2ERRINT 83
#define IRQ_EDMA3C3ERRINT 84
#if defined(CONFIG_SOC_TMS320C6457)
#define IRQ_EDMA3CCAET    85
#define IRQ_EDMA3C4ERRINT 86
#define IRQ_EDMA3C5ERRINT 86
#define IRQ_ETBOVFLINT    94
#define IRQ_ETBUNFLINT    95
#endif
#define IRQ_INTERR        96  /* interrupt controller dropped CPU interrupt event */
#define IRQ_EMCIDMAERR    97  /* EMC invalid IDMA parameters */
#if defined(CONFIG_SOC_TMS320C6457)
#define IRQ_EFIINTA       100
#define IRQ_EFIINTB       101
#define IRQ_L1PED1        113
#define IRQ_L1ED1         116
#define IRQ_L2ED1         117
#define IRQ_PDCINT        118
#endif
#define IRQ_L1PCMPA       120 /* L1P CPU memory protection fault */
#define IRQ_L1PDMPA       121 /* L1P DMA memory protection fault */
#define IRQ_L1DCMPA       122 /* L1D CPU memory protection fault */
#define IRQ_L1DDMPA       123 /* L1D DMA memory protection fault */
#define IRQ_L2CMPA        124 /* L2 CPU memory protection fault */
#define IRQ_L2CDMPA       125 /* L2 DMA memory protection fault */
#define IRQ_IDMACMPA      126 /* IDMA CPU memory protection fault */
#define IRQ_IDMABUSERR    127 /* IDMA bus error fault */

#define NR_SOC_IRQS	  128

#endif /* __MACH_IRQ_C645X_H */
