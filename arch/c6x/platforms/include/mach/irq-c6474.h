/*
 *  arch/c6x/platforms/include/mach/irq-c6474.h
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
#ifndef __MACH_IRQ_C6474_H
#define __MACH_IRQ_C6474_H

#define IRQ_EVT0        0   /* combined events */
#define IRQ_EVT1        1
#define IRQ_EVT2        2
#define IRQ_EVT3        3
#define IRQ_SEMINT      4   /* semaphore grant interrupt */
#define IRQ_EMACINT     5   /* EMAC interrupts */
#define IRQ_EMACRXINT   6
#define IRQ_EMACTXINT   7
#define IRQ_EMACTHRES   8
#define IRQ_EMUDTDMA    9   /* emulator events */
#define IRQ_RACINT      10  /* RAC interrupt */
#define IRQ_EMURTDXRX   11
#define IRQ_EMURTDXTX   12
#define IRQ_IDMAINT0    13  /* IDMA channel 0 interrupt */
#define IRQ_IDMAINT1    14  /* IDMA channel 1 interrupt */
#define IRQ_FSEVT0      15  /* frame synchronization events */
#define IRQ_FSEVT1      16
#define IRQ_FSEVT2      17
#define IRQ_FSEVT3      18
#define IRQ_FSEVT4      19
#define IRQ_FSEVT5      29
#define IRQ_FSEVT6      21
#define IRQ_FSEVT7      22
#define IRQ_FSEVT8      23
#define IRQ_FSEVT9      24
#define IRQ_FSEVT10     25
#define IRQ_FSEVT11     26
#define IRQ_FSEVT12     27
#define IRQ_FSEVT13     28
#define IRQ_FSEVT14     29
#define IRQ_FSEVT15     30
#define IRQ_FSEVT16     31
#define IRQ_FSEVT17     32
#define IRQ_TINT0       33  /* timer interrupts */
#define IRQ_TINTLO0     33
#define IRQ_TINTHI0     34
#define IRQ_TINT1       35
#define IRQ_TINTLO1     35
#define IRQ_TINTHI1     36
#define IRQ_TINT2       37
#define IRQ_TINTLO2     37
#define IRQ_TINTHI2     38
#define IRQ_TINT3       39
#define IRQ_TINTLO3     39
#define IRQ_TINTHI3     40
#define IRQ_TINT4       41
#define IRQ_TINTLO4     41
#define IRQ_TINTHI4     42
#define IRQ_TINT5       43
#define IRQ_TINTLO5     43
#define IRQ_TINTHI5     44
#define IRQ_GPIO_START  45
#define IRQ_GPIO0       45  /* GPIO events */
#define IRQ_GPIO1       46
#define IRQ_GPIO2       47
#define IRQ_GPIO3       48
#define IRQ_GPIO4       49
#define IRQ_GPIO5       50
#define IRQ_GPIO6       51
#define IRQ_GPIO7       52
#define IRQ_GPIO8       53
#define IRQ_GPIO9       54
#define IRQ_GPIO10      55
#define IRQ_GPIO11      56
#define IRQ_GPIO12      57
#define IRQ_GPIO13      58
#define IRQ_GPIO14      59
#define IRQ_GPIO15      60
#define IRQ_TPCCGINT    61  /* EDMA channel global completion interrupt */
#define IRQ_TPCCINT0    62  /* TPCC completion interrupts */
#define IRQ_TPCCINT1    63
#define IRQ_TPCCINT2    64
#define IRQ_TPCCINT3    65
#define IRQ_TPCCINT4    66
#define IRQ_TPCCINT5    67
#define IRQ_TPCCINT6    68
#define IRQ_TPCCINT7    69
#define IRQ_RIOINT0     71  /* RapidIO interrupts */
#define IRQ_RIOINT1     72
#define IRQ_AIFEVT0     73  /* error/alarm events*/
#define IRQ_AIFEVT1     74
#define IRQ_IPCLOCAL    76  /* inter DSP interrupt from IPCGR */

#define CIC_MAPBASE	80  /* start of system events from CIC */
#define CIC_MAPLEN	14  /* number of events from CIC */

#define IRQ_CICEVT0	80
#define IRQ_CICEVT1	81
#define IRQ_CICEVT2	82
#define IRQ_CICEVT3	83
#define IRQ_CICEVT4	84
#define IRQ_CICEVT5	85
#define IRQ_CICEVT6	86
#define IRQ_CICEVT7	87
#define IRQ_CICEVT8	88
#define IRQ_CICEVT9	89
#define IRQ_CICEVT10	90
#define IRQ_CICEVT11	91
#define IRQ_CICEVT12	92
#define IRQ_CICEVT13	93

#define IRQ_INTERR      96  /* irq controller dropped CPU interrupt event */
#define IRQ_EMCIDMAERR  97  /* EMC invalid IDMA parameters */
#define IRQ_EFINTA      100 /* EFI interrupt from side A */
#define IRQ_EFINTB      101 /* EFI interrupt from side B */
#define IRQ_PMCED       113 /* single bit error detected during DMA read */
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

/*
 * C6x Chip Interrupt Controller (CIC) events
 */ 
#define IRQ_CIC_START	128
#define IRQ_I2CINT      (IRQ_CIC_START + 4)  /* I2C event */
#define IRQ_FSERR1      (IRQ_CIC_START + 5)  /* error/alarm interrupts */
#define IRQ_RIOINT7     (IRQ_CIC_START + 6)
#define IRQ_FSERR2      (IRQ_CIC_START + 7)
#define IRQ_VCPINT      (IRQ_CIC_START + 8)
#define IRQ_TCPINT      (IRQ_CIC_START + 9)
#define IRQ_RINT0       (IRQ_CIC_START + 10) /* McBSP event */
#define IRQ_XINT0       (IRQ_CIC_START + 11)
#define IRQ_RINT1       (IRQ_CIC_START + 12)
#define IRQ_XINT1       (IRQ_CIC_START + 13)
#define IRQ_REVT0       (IRQ_CIC_START + 14)
#define IRQ_XEVT0       (IRQ_CIC_START + 15)
#define IRQ_REVT1       (IRQ_CIC_START + 16)
#define IRQ_XEVT1       (IRQ_CIC_START + 17)
#define IRQ_IREVT1      (IRQ_CIC_START + 18) /* I2C EDMA events */
#define IRQ_IXEVT1      (IRQ_CIC_START + 19)
#define IRQ_FSEVT18     (IRQ_CIC_START + 20) /* frame synchronization events */
#define IRQ_FSEVT19     (IRQ_CIC_START + 21)
#define IRQ_FSEVT20     (IRQ_CIC_START + 22)
#define IRQ_FSEVT21     (IRQ_CIC_START + 23)
#define IRQ_FSEVT22     (IRQ_CIC_START + 24)
#define IRQ_FSEVT23     (IRQ_CIC_START + 25)
#define IRQ_FSEVT24     (IRQ_CIC_START + 26)
#define IRQ_FSEVT25     (IRQ_CIC_START + 27)
#define IRQ_FSEVT26     (IRQ_CIC_START + 28)
#define IRQ_FSEVT27     (IRQ_CIC_START + 29)
#define IRQ_FSEVT28     (IRQ_CIC_START + 30)
#define IRQ_FSEVT29     (IRQ_CIC_START + 31)
#define IRQ_VCPREVT     (IRQ_CIC_START + 32) /* VCP events */
#define IRQ_VCPXEVT     (IRQ_CIC_START + 33)
#define IRQ_TCPREVT     (IRQ_CIC_START + 34) /* TCP events */
#define IRQ_TCPXEVT     (IRQ_CIC_START + 35)
#define IRQ_TPCCERRINT  (IRQ_CIC_START + 36) /* TPCC error interrupt */
#define IRQ_TPCCMPINT   (IRQ_CIC_START + 37) /* TPCSS memory protection interrupt */
#define IRQ_TPTCERRINT0 (IRQ_CIC_START + 38) /* TPTC error interrupts */
#define IRQ_TPTCERRINT1 (IRQ_CIC_START + 39)
#define IRQ_TPTCERRINT2 (IRQ_CIC_START + 40)
#define IRQ_TPTCERRINT3 (IRQ_CIC_START + 41)
#define IRQ_TPTCERRINT4 (IRQ_CIC_START + 42)
#define IRQ_TPTCERRINT5 (IRQ_CIC_START + 43)
#define IRQ_TPTCAETEVT  (IRQ_CIC_START + 44) /* TPTC AET event */
#define IRQ_AIFEVT2     (IRQ_CIC_START + 45) /* AIF CPU interrupts */
#define IRQ_AIFEVT3     (IRQ_CIC_START + 46)
#define IRQ_AIFPSEVT0   (IRQ_CIC_START + 47) /* packet switched transfer events */
#define IRQ_AIFPSEVT1   (IRQ_CIC_START + 48)
#define IRQ_AIFPSEVT2   (IRQ_CIC_START + 49)
#define IRQ_AIFPSEVT3   (IRQ_CIC_START + 50)
#define IRQ_AIFPSEVT4   (IRQ_CIC_START + 51)
#define IRQ_AIFPSEVT5   (IRQ_CIC_START + 52)
#define IRQ_AIFPSEVT6   (IRQ_CIC_START + 53)
#define IRQ_AIFBUFEVT   (IRQ_CIC_START + 54)
#define IRQ_RACDEVENT0  (IRQ_CIC_START + 56) /* debug events */
#define IRQ_RACDEVENT1  (IRQ_CIC_START + 57)
#define IRQ_SEMERR      (IRQ_CIC_START + 58)

#define NR_CIC_IRQS	64

#define NR_SOC_IRQS	(IRQ_CIC_START + NR_CIC_IRQS)

/*
 * C6x Chip Interrupt Controller (CIC) register layout
 */
#define CIC_REG_BASE(corenum) (0x02880000 + (0x100 * (corenum)))

#define CIC_EVTFLAG(n)	__SYSREGA(CIC_REG_BASE(n) + 0x00, uint32_t)
#define CIC_EVTSET(n)	__SYSREGA(CIC_REG_BASE(n) + 0x10, uint32_t)
#define CIC_EVTCLR(n)	__SYSREGA(CIC_REG_BASE(n) + 0x20, uint32_t)
#define CIC_EVTMASK(n)	__SYSREGA(CIC_REG_BASE(n) + 0x30, uint32_t)
#define CIC_MEVTFLAG(n)	__SYSREGA(CIC_REG_BASE(n) + 0x40, uint32_t)
#define CIC_MUX(n)	__SYSREGA(CIC_REG_BASE(n) + 0x50, uint32_t)

#define CIC0_EVTFLAG	__SYSREGA(CIC_REG_BASE(0) + 0x00, uint32_t)
#define CIC0_EVTSET	__SYSREGA(CIC_REG_BASE(0) + 0x10, uint32_t)
#define CIC0_EVTCLR	__SYSREGA(CIC_REG_BASE(0) + 0x20, uint32_t)
#define CIC0_EVTMASK	__SYSREGA(CIC_REG_BASE(0) + 0x30, uint32_t)
#define CIC0_MEVTFLAG	__SYSREGA(CIC_REG_BASE(0) + 0x40, uint32_t)
#define CIC0_MUX	__SYSREGA(CIC_REG_BASE(0) + 0x50, uint32_t)

#define CIC1_EVTFLAG	__SYSREGA(CIC_REG_BASE(1) + 0x00, uint32_t)
#define CIC1_EVTSET	__SYSREGA(CIC_REG_BASE(1) + 0x10, uint32_t)
#define CIC1_EVTCLR	__SYSREGA(CIC_REG_BASE(1) + 0x20, uint32_t)
#define CIC1_EVTMASK	__SYSREGA(CIC_REG_BASE(1) + 0x30, uint32_t)
#define CIC1_MEVTFLAG	__SYSREGA(CIC_REG_BASE(1) + 0x40, uint32_t)
#define CIC1_MUX	__SYSREGA(CIC_REG_BASE(1) + 0x50, uint32_t)

#define CIC2_EVTFLAG	__SYSREGA(CIC_REG_BASE(2) + 0x00, uint32_t)
#define CIC2_EVTSET	__SYSREGA(CIC_REG_BASE(2) + 0x10, uint32_t)
#define CIC2_EVTCLR	__SYSREGA(CIC_REG_BASE(2) + 0x20, uint32_t)
#define CIC2_EVTMASK	__SYSREGA(CIC_REG_BASE(2) + 0x30, uint32_t)
#define CIC2_MEVTFLAG	__SYSREGA(CIC_REG_BASE(2) + 0x40, uint32_t)
#define CIC2_MUX	__SYSREGA(CIC_REG_BASE(2) + 0x50, uint32_t)

#define CIC3_EVTFLAG	__SYSREGA(CIC_REG_BASE(3) + 0x00, uint32_t)
#define CIC3_EVTSET	__SYSREGA(CIC_REG_BASE(3) + 0x10, uint32_t)
#define CIC3_EVTCLR	__SYSREGA(CIC_REG_BASE(3) + 0x20, uint32_t)
#define CIC3_EVTMASK	__SYSREGA(CIC_REG_BASE(3) + 0x30, uint32_t)
#define CIC3_MEVTFLAG	__SYSREGA(CIC_REG_BASE(3) + 0x40, uint32_t)
#define CIC3_MUX	__SYSREGA(CIC_REG_BASE(3) + 0x50, uint32_t)

/*
 * CIC output events (16)
 */
#define CIC0               0
#define CIC1               1
#define CIC2               2
#define CIC3               3
#define CIC4               4
#define CIC5               5
#define CIC6               6
#define CIC7               7
#define CIC8               8
#define CIC9               9
#define CIC10              10
#define CIC11              11
#define CIC12              12
#define CIC13              13
#define CIC14              14
#define CIC15              15

/*
 * CIC events for CIC3 (CIC TPCC) 
 */
#define CIC_TPCC          3   /* CIC TPPC is CIC3 */

#define CIC_TPCC_EVT0     0   /* combined events */
#define CIC_TPCC_EVT1     1
#define CIC_TPCC_FSEVT0   2   /* frame synchronization events */
#define CIC_TPCC_FSEVT1   3
#define CIC_TPCC_FSEVT2   4
#define CIC_TPCC_FSEVT3   5
#define CIC_TPCC_FSEVT14  6
#define CIC_TPCC_FSEVT15  7
#define CIC_TPCC_FSEVT16  8
#define CIC_TPCC_FSEVT17  9
#define CIC_TPCC_FSEVT18  10
#define CIC_TPCC_FSEVT19  11
#define CIC_TPCC_FSEVT20  12
#define CIC_TPCC_FSEVT21  13
#define CIC_TPCC_FSEVT22  14
#define CIC_TPCC_FSEVT23  15
#define CIC_TPCC_FSEVT24  16
#define CIC_TPCC_FSEVT25  17
#define CIC_TPCC_FSEVT26  18
#define CIC_TPCC_FSEVT27  19
#define CIC_TPCC_FSEVT28  20
#define CIC_TPCC_RIOINT0  21
#define CIC_TPCC_RIOINT1  22
#define CIC_TPCC_RIOINT2  23
#define CIC_TPCC_RIOINT3  24
#define CIC_TPCC_RIOINT4  25
#define CIC_TPCC_RIOINT5  26
#define CIC_TPCC_RIOINT7  27
#define CIC_TPCC_MACINT0  28
#define CIC_TPCC_MACRINT0 29
#define CIC_TPCC_MACTINT0 30
#define CIC_TPCC_MACINT1  31
#define CIC_TPCC_MACRINT1 32
#define CIC_TPCC_MACTINT1 33
#define CIC_TPCC_MACINT2  34
#define CIC_TPCC_MACRINT2 35
#define CIC_TPCC_MACTINT2 36
#define CIC_TPCC_SEMERR0  37
#define CIC_TPCC_SEMERR1  38
#define CIC_TPCC_SEMERR2  39
#define CIC_TPCC_TINT3L   43
#define CIC_TPCC_TINT3H   44
#define CIC_TPCC_TINT4L   45
#define CIC_TPCC_TINT4H   46
#define CIC_TPCC_TINT5L   47
#define CIC_TPCC_TINT5H   48
#define CIC_TPCC_AIFTEVT0 49
#define CIC_TPCC_AIFTEVT1 50
#define CIC_TPCC_GPINT0   53
#define CIC_TPCC_GPINT1   54
#define CIC_TPCC_GPINT2   55
#define CIC_TPCC_GPINT3   56
#define CIC_TPCC_GPINT4   57
#define CIC_TPCC_CIC0E14  58
#define CIC_TPCC_CIC0E15  59
#define CIC_TPCC_CIC1E14  60
#define CIC_TPCC_CIC1E15  61
#define CIC_TPCC_CIC2E14  62
#define CIC_TPCC_CIC2E15  63

#endif /* __MACH_IRQ_C6474_H */
