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

/*
 * Interrupt selector registers
 */
#define IRQ_EVTFLAG0_REG  0x01800000
#define IRQ_EVTFLAG1_REG  0x01800004
#define IRQ_EVTFLAG2_REG  0x01800008
#define IRQ_EVTFLAG3_REG  0x0180000c
#define IRQ_EVTSET0_REG   0x01800020
#define IRQ_EVTSET1_REG   0x01800024
#define IRQ_EVTSET2_REG   0x01800028
#define IRQ_EVTSET3_REG   0x0180002c
#define IRQ_EVTCLR0_REG   0x01800040
#define IRQ_EVTCLR1_REG   0x01800044
#define IRQ_EVTCLR2_REG   0x01800048
#define IRQ_EVTCLR3_REG   0x0180004c
#define IRQ_EVTMASK0_REG  0x01800080
#define IRQ_EVTMASK1_REG  0x01800084
#define IRQ_EVTMASK2_REG  0x01800088
#define IRQ_EVTMASK3_REG  0x0180008c
#define IRQ_MEVTFLAG0_REG 0x018000a0
#define IRQ_MEVTFLAG1_REG 0x018000a4
#define IRQ_MEVTFLAG2_REG 0x018000a8
#define IRQ_MEVTFLAG3_REG 0x018000ac
#define IRQ_EXPMASK0_REG  0x018000c0
#define IRQ_EXPMASK1_REG  0x018000c4
#define IRQ_EXPMASK2_REG  0x018000c8
#define IRQ_EXPMASK3_REG  0x018000cc
#define IRQ_MEXPMASK0_REG 0x018000e0
#define IRQ_MEXPMASK1_REG 0x018000e4
#define IRQ_MEXPMASK2_REG 0x018000e8
#define IRQ_MEXPMASK3_REG 0x018000ec
#define IRQ_INTMUX1_REG   0x01800104
#define IRQ_INTMUX2_REG   0x01800108
#define IRQ_INTMUX3_REG   0x0180010c
#define IRQ_AEGMUX0_REG   0x01800140
#define IRQ_AEGMUX1_REG   0x01800144
#define IRQ_INTXSTAT_REG  0x01800180
#define IRQ_INTXCLR_REG   0x01800184
#define IRQ_INTDMASK_REG  0x01800188
#define IRQ_EVTASRT_REG   0x018001c0

/* 
 * EVTASRT bits
 */
#define IRQ_B_EVTPULSE4   (1 << 4)

#define IRQ_EVT0          0   /* combined events */
#define IRQ_EVT1          1
#define IRQ_EVT2          2
#define IRQ_EVT3          3
#define IRQ_SEMINT        4   /* semaphore grant interrupt */
#define IRQ_EMACINT       5   /* EMAC interrupts */
#define IRQ_EMACRXINT     6
#define IRQ_EMACTXINT     7
#define IRQ_EMACTHRES     8   
#define IRQ_EMUDTDMA      9   /* emulator events */
#define IRQ_RACINT        10  /* RAC interrupt */
#define IRQ_EMURTDXRX     11
#define IRQ_EMURTDXTX     12
#define IRQ_IDMAINT0      13  /* IDMA channel 0 interrupt */
#define IRQ_IDMAINT1      14  /* IDMA channel 1 interrupt */
#define IRQ_FSEVT0        15  /* frame synchronization events */
#define IRQ_FSEVT1        16
#define IRQ_FSEVT2        17
#define IRQ_FSEVT3        18
#define IRQ_FSEVT4        19
#define IRQ_FSEVT5        29
#define IRQ_FSEVT6        21
#define IRQ_FSEVT7        22
#define IRQ_FSEVT8        23
#define IRQ_FSEVT9        24
#define IRQ_FSEVT10       25
#define IRQ_FSEVT11       26
#define IRQ_FSEVT12       27
#define IRQ_FSEVT13       28
#define IRQ_FSEVT14       29
#define IRQ_FSEVT15       30
#define IRQ_FSEVT16       31
#define IRQ_FSEVT17       32
#define IRQ_TINT0         33  /* timer interrupts */
#define IRQ_TINTLO0       33
#define IRQ_TINTHI0       34
#define IRQ_TINT1         35
#define IRQ_TINTLO1       35
#define IRQ_TINTHI1       36
#define IRQ_TINT2         37
#define IRQ_TINTLO2       37
#define IRQ_TINTHI2       38
#define IRQ_TINT3         39
#define IRQ_TINTLO3       39
#define IRQ_TINTHI3       40
#define IRQ_TINT4         41
#define IRQ_TINTLO4       41
#define IRQ_TINTHI4       42
#define IRQ_TINT5         43
#define IRQ_TINTLO5       43
#define IRQ_TINTHI5       44
#define IRQ_GPIO0         45  /* GPIO events */
#define IRQ_GPIO1         46
#define IRQ_GPIO2         47
#define IRQ_GPIO3         48
#define IRQ_GPIO4         49
#define IRQ_GPIO5         50
#define IRQ_GPIO6         51
#define IRQ_GPIO7         52
#define IRQ_GPIO8         53
#define IRQ_GPIO9         54
#define IRQ_GPIO10        55
#define IRQ_GPIO11        56
#define IRQ_GPIO12        57
#define IRQ_GPIO13        58
#define IRQ_GPIO14        59
#define IRQ_GPIO15        60
#define IRQ_TPCCGINT      61  /* EDMA channel global completion interrupt */
#define IRQ_TPCCINT0      62  /* TPCC completion interrupts */
#define IRQ_TPCCINT1      63
#define IRQ_TPCCINT2      64
#define IRQ_TPCCINT3      65
#define IRQ_TPCCINT4      66
#define IRQ_TPCCINT5      67
#define IRQ_TPCCINT6      68
#define IRQ_TPCCINT7      69
#define IRQ_RIOINT1       71  /* RapidIO interrupts */
#define IRQ_RIOINT2       72
#define IRQ_AIFEVT0       73  /* error/alarm events*/
#define IRQ_AIFEVT1       74 
#define IRQ_IPCLOCAL      76  /* inter DSP interrupt from IPCGR */
#define IRQ_CICEVT0       80  /* system events from CIC */
#define IRQ_CICEVT1       81
#define IRQ_CICEVT2       82
#define IRQ_CICEVT3       83
#define IRQ_CICEVT4       84
#define IRQ_CICEVT5       85
#define IRQ_CICEVT6       86
#define IRQ_CICEVT7       87
#define IRQ_CICEVT8       88
#define IRQ_CICEVT9       89
#define IRQ_CICEVT10      90
#define IRQ_CICEVT11      91
#define IRQ_CICEVT12      92
#define IRQ_CICEVT13      93
#define IRQ_INTERR        96  /* interrupt controller dropped CPU interrupt event */
#define IRQ_EMCIDMAERR    97  /* EMC invalid IDMA parameters */
#define IRQ_EFINTA        100 /* EFI interrupt from side A */
#define IRQ_EFINTB        101 /* EFI interrupt from side B */
#define IRQ_PMCED         113 /* single bit error detected during DMA read */
#define IRQ_UMCED1        116 /* corrected bit error detected */
#define IRQ_UMCED2        117 /* uncorrected bit error detected */
#define IRQ_PDCINT        118 /* PDC sleep interrupt */
#define IRQ_SYSCMPA       119 /* CPU memory protection fault */
#define IRQ_L1PCMPA       120 /* L1P CPU memory protection fault */
#define IRQ_L1PDMPA       121 /* L1P DMA memory protection fault */
#define IRQ_L1DCMPA       122 /* L1D CPU memory protection fault */
#define IRQ_L1DDMPA       123 /* L1D DMA memory protection fault */
#define IRQ_L2CMPA        124 /* L2 CPU memory protection fault */
#define IRQ_L2CDMPA       125 /* L2 DMA memory protection fault */
#define IRQ_EMCCMPA       126 /* external CPU memory protection fault */
#define IRQ_EMCBUSERR     127 /* bus error interrupt */

/*
 * C6x Chip Interrupt Controller (CIC) events
 */ 
#define CIC_EVT0          0   /* combined events */ 
#define CIC_EVT1          1
#define CIC_IICINT        4   /* I2C event */
#define CIC_FSERR1        5   /* error/alarm interrupts */ 
#define CIC_RIOINT7       6   
#define CIC_FSERR2        7
#define CIC_VCPINT        8
#define CIC_TCPINT        9
#define CIC_RINT0         10  /* McBSP event */
#define CIC_XINT0         11
#define CIC_RINT1         12
#define CIC_XINT1         13
#define CIC_REVT0         14
#define CIC_XEVT0         15
#define CIC_REVT1         16
#define CIC_XEVT1         17
#define CIC_IREVT1        18  /* I2C EDMA events */
#define CIC_IXEVT1        19
#define CIC_FSEVT18       20  /* frame synchronization events */
#define CIC_FSEVT19       21
#define CIC_FSEVT20       22
#define CIC_FSEVT21       23
#define CIC_FSEVT22       24
#define CIC_FSEVT23       25
#define CIC_FSEVT24       26
#define CIC_FSEVT25       27
#define CIC_FSEVT26       28
#define CIC_FSEVT27       29
#define CIC_FSEVT28       30
#define CIC_FSEVT29       31
#define CIC_VCPREVT       32  /* VCP events */
#define CIC_VCPXEVT       33
#define CIC_TCPREVT       34  /* TCP events */
#define CIC_TCPXEVT       35
#define CIC_TPCCERRINT    36  /* TPCC error interrupt */
#define CIC_TPCCMPINT     37  /* TPCSS memory protection interrupt */
#define CIC_TPTCERRINT0   38  /* TPTC error interrupts */
#define CIC_TPTCERRINT1   39  
#define CIC_TPTCERRINT2   40
#define CIC_TPTCERRINT3   41
#define CIC_TPTCERRINT4   42
#define CIC_TPTCERRINT5   43
#define CIC_TPTCAETEVT    44  /* TPTC AET event */
#define CIC_AIFEVT2       45  /* AIF CPU interrupts */
#define CIC_AIFEVT3       46
#define CIC_AIFPSEVT0     47  /* packet switched transfer events */
#define CIC_AIFPSEVT1     48
#define CIC_AIFPSEVT2     49
#define CIC_AIFPSEVT3     50
#define CIC_AIFPSEVT4     51
#define CIC_AIFPSEVT5     52
#define CIC_AIFPSEVT6     53
#define CIC_AIFBUFEVT     54
#define CIC_RACDEVENT0    56  /* debug events */
#define CIC_RACDEVENT1    57
#define CIC_SEMERR        58

#endif /* __MACH_IRQ_C6474_H */
