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
#define IRQ_PCIEMSI0    17  /* PCIe MSI */
#define IRQ_RIOINT16    20  /* RapidIO interrupts */
#define IRQ_RIOINT20    21
#define IRQ_INTC0OUT    22  /* INTC per-core outputs: 22 to 31 */
#define IRQ_QML         32  /* QM low: 32 to 47 */
#define IRQ_QMH         48  /* QM high: 48 to 55 */
#define IRQ_INTC0OUT0   56  /* CP_INTC outputs */
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
#define IRQ_INTC0OUT8   74  /* CP_INTC outputs */
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
#define IRQ_INTC0OUT12  92  /* CP_INTC outputs */
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

/*
 * CP_INTC0 events
 */ 
#define IRQ_CPINTC0_START    128
#define IRQ_TPCC1ERRINT      (IRQ_CPINTC0_START)      /* TPCC1 error */
#define IRQ_TPCC1MPINT       (IRQ_CPINTC0_START + 1)  /* TPCC1 mem protect */
#define IRQ_TPTC1ERRINT0     (IRQ_CPINTC0_START + 2)  /* TPCC1 TPTC0 error */
#define IRQ_TPTC1ERRINT1     (IRQ_CPINTC0_START + 3)  /* TPCC1 TPTC1 error */
#define IRQ_TPTC1ERRINT2     (IRQ_CPINTC0_START + 4)  /* TPCC1 TPTC2 error */
#define IRQ_TPTC1ERRINT3     (IRQ_CPINTC0_START + 5)  /* TPCC1 TPTC3 error */
#define IRQ_TPCC1GINT        (IRQ_CPINTC0_START + 6)  /* TPCC1 GINT */
#define IRQ_TPCC1INT0        (IRQ_CPINTC0_START + 8)  /* TPCC1 completion interrupts */
#define IRQ_TPCC1INT1        (IRQ_CPINTC0_START + 9)
#define IRQ_TPCC1INT2        (IRQ_CPINTC0_START + 10)
#define IRQ_TPCC1INT3        (IRQ_CPINTC0_START + 11)
#define IRQ_TPCC1INT4        (IRQ_CPINTC0_START + 12)
#define IRQ_TPCC1INT5        (IRQ_CPINTC0_START + 13)
#define IRQ_TPCC1INT6        (IRQ_CPINTC0_START + 14)
#define IRQ_TPCC1INT7        (IRQ_CPINTC0_START + 15)
#define IRQ_TPCC2ERRINT      (IRQ_CPINTC0_START + 16)  /* TPCC2 error */
#define IRQ_TPCC2MPINT       (IRQ_CPINTC0_START + 17)  /* TPCC2 mem protect */
#define IRQ_TPTC2ERRINT0     (IRQ_CPINTC0_START + 18)  /* TPCC2 TPTC0 error */
#define IRQ_TPTC2ERRINT1     (IRQ_CPINTC0_START + 19)  /* TPCC2 TPTC1 error */
#define IRQ_TPTC2ERRINT2     (IRQ_CPINTC0_START + 20)  /* TPCC2 TPTC2 error */
#define IRQ_TPTC2ERRINT3     (IRQ_CPINTC0_START + 21)  /* TPCC2 TPTC3 error */
#define IRQ_TPCC2GINT        (IRQ_CPINTC0_START + 22)  /* TPCC2 GINT */
#define IRQ_TPCC2INT0        (IRQ_CPINTC0_START + 24)  /* TPCC2 completion interrupts */
#define IRQ_TPCC2INT1        (IRQ_CPINTC0_START + 25)
#define IRQ_TPCC2INT2        (IRQ_CPINTC0_START + 26)
#define IRQ_TPCC2INT3        (IRQ_CPINTC0_START + 27)
#define IRQ_TPCC2INT4        (IRQ_CPINTC0_START + 28)
#define IRQ_TPCC2INT5        (IRQ_CPINTC0_START + 29)
#define IRQ_TPCC2INT6        (IRQ_CPINTC0_START + 30)
#define IRQ_TPCC2INT7        (IRQ_CPINTC0_START + 31)
#define IRQ_TPCC0ERRINT      (IRQ_CPINTC0_START + 32)  /* TPCC0 error */
#define IRQ_TPCC0MPINT       (IRQ_CPINTC0_START + 33)  /* TPCC0 mem protect */
#define IRQ_TPTC0ERRINT0     (IRQ_CPINTC0_START + 34)  /* TPCC0 TPTC0 error */
#define IRQ_TPTC0ERRINT1     (IRQ_CPINTC0_START + 35)  /* TPCC0 TPTC1 error */
#define IRQ_TPCC0GINT        (IRQ_CPINTC0_START + 36)  /* TPCC0 GINT */
#define IRQ_TPCC0INT0        (IRQ_CPINTC0_START + 38)  /* TPCC0 completion interrupts */
#define IRQ_TPCC0INT1        (IRQ_CPINTC0_START + 39)
#define IRQ_TPCC0INT2        (IRQ_CPINTC0_START + 40)
#define IRQ_TPCC0INT3        (IRQ_CPINTC0_START + 41)
#define IRQ_TPCC0INT4        (IRQ_CPINTC0_START + 42)
#define IRQ_TPCC0INT5        (IRQ_CPINTC0_START + 43)
#define IRQ_TPCC0INT6        (IRQ_CPINTC0_START + 44)
#define IRQ_TPCC0INT7        (IRQ_CPINTC0_START + 45)
#define IRQ_PCIEERR          (IRQ_CPINTC0_START + 48)  /* PCIe error */
#define IRQ_PCIEPM           (IRQ_CPINTC0_START + 49)  /* PCIe PM */
#define IRQ_PCIEINTA         (IRQ_CPINTC0_START + 50)  /* PCIe legacy interrupts */
#define IRQ_PCIEINTB         (IRQ_CPINTC0_START + 51)
#define IRQ_PCIEINTC         (IRQ_CPINTC0_START + 52)
#define IRQ_PCIEINTD         (IRQ_CPINTC0_START + 53)
#define IRQ_SPIINT0          (IRQ_CPINTC0_START + 54)  /* SPI interrupts */
#define IRQ_SPIINT1          (IRQ_CPINTC0_START + 55)
#define IRQ_SPIXEVT          (IRQ_CPINTC0_START + 56)
#define IRQ_SPIREVT          (IRQ_CPINTC0_START + 57)
#define IRQ_I2CINT           (IRQ_CPINTC0_START + 58)  /* I2C interrupt */
#define IRQ_I2CREVT          (IRQ_CPINTC0_START + 59)
#define IRQ_I2CXEVT          (IRQ_CPINTC0_START + 60)
#define IRQ_C0TETBHFULL      (IRQ_CPINTC0_START + 63)  /* TETB */
#define IRQ_C0TETBFULL       (IRQ_CPINTC0_START + 64)
#define IRQ_C0TETBAXQ        (IRQ_CPINTC0_START + 65)
#define IRQ_C0TETBOVFL       (IRQ_CPINTC0_START + 66)
#define IRQ_C0TETBUNFL       (IRQ_CPINTC0_START + 67)
#define IRQ_MDIOLNKINT0      (IRQ_CPINTC0_START + 68)  /* PA MDIO interrupts */
#define IRQ_MDIOLNKINT1      (IRQ_CPINTC0_START + 69)
#define IRQ_MDIOUSRINT0      (IRQ_CPINTC0_START + 70)
#define IRQ_MDIOUSRINT1      (IRQ_CPINTC0_START + 71)
#define IRQ_PAMISC           (IRQ_CPINTC0_START + 72)
#define IRQ_TRACEC0          (IRQ_CPINTC0_START + 73)  /* tracer interrupts */
#define IRQ_TRACEC1          (IRQ_CPINTC0_START + 74)
#define IRQ_TRACEC2          (IRQ_CPINTC0_START + 75)
#define IRQ_TRACEC3          (IRQ_CPINTC0_START + 76)
#define IRQ_TRACEDDR         (IRQ_CPINTC0_START + 77)
#define IRQ_TRACEMSMC0       (IRQ_CPINTC0_START + 78)
#define IRQ_TRACEMSMC1       (IRQ_CPINTC0_START + 79)
#define IRQ_TRACEMSMC2       (IRQ_CPINTC0_START + 80)
#define IRQ_TRACEMSMC3       (IRQ_CPINTC0_START + 81)
#define IRQ_TRACECFG         (IRQ_CPINTC0_START + 82)
#define IRQ_TRACEQMCFG       (IRQ_CPINTC0_START + 83)
#define IRQ_TRACEQMDMA       (IRQ_CPINTC0_START + 84)
#define IRQ_TRACESEM         (IRQ_CPINTC0_START + 85)
#define IRQ_PSCALL           (IRQ_CPINTC0_START + 86)  /* PSC */
#define IRQ_MSMCSCRUB        (IRQ_CPINTC0_START + 87)  /* MSMC scrub error */
#define IRQ_BOOTCFG          (IRQ_CPINTC0_START + 88)  /* MMR error */
#define IRQ_MPU0             (IRQ_CPINTC0_START + 90)  /* MPU */
#define IRQ_MPU1             (IRQ_CPINTC0_START + 92)
#define IRQ_MPU2             (IRQ_CPINTC0_START + 94)
#define IRQ_MPU3             (IRQ_CPINTC0_START + 96)
#define IRQ_MSMCDEDC         (IRQ_CPINTC0_START + 98)  /* MSMC */
#define IRQ_MSMCDEDCNC       (IRQ_CPINTC0_START + 99)
#define IRQ_MSMCSCRUBNC      (IRQ_CPINTC0_START + 100)
#define IRQ_MSMCMPAX         (IRQ_CPINTC0_START + 101)
#define IRQ_MSMCMPFINT8      (IRQ_CPINTC0_START + 102)
#define IRQ_MSMCMPFINT9      (IRQ_CPINTC0_START + 103)
#define IRQ_MSMCMPFINT10     (IRQ_CPINTC0_START + 104)
#define IRQ_MSMCMPFINT11     (IRQ_CPINTC0_START + 105)
#define IRQ_MSMCMPFINT12     (IRQ_CPINTC0_START + 106)
#define IRQ_MSMCMPFINT13     (IRQ_CPINTC0_START + 107)
#define IRQ_MSMCMPFINT14     (IRQ_CPINTC0_START + 108)
#define IRQ_MSMCMPFINT15     (IRQ_CPINTC0_START + 109)
#define IRQ_DDR3             (IRQ_CPINTC0_START + 110) /* DDR3 EMIF error */
#define IRQ_HYPERLINK        (IRQ_CPINTC0_START + 111) /* HyperLink */
#define IRQ_RIOINT0          (IRQ_CPINTC0_START + 112) /* RapidIO  */
#define IRQ_RIOINT1          (IRQ_CPINTC0_START + 113)
#define IRQ_RIOINT2          (IRQ_CPINTC0_START + 114)
#define IRQ_RIOINT3          (IRQ_CPINTC0_START + 115)
#define IRQ_RIOINT4          (IRQ_CPINTC0_START + 116)
#define IRQ_RIOINT5          (IRQ_CPINTC0_START + 117)
#define IRQ_RIOINT6          (IRQ_CPINTC0_START + 118)
#define IRQ_RIOINT7          (IRQ_CPINTC0_START + 119)
#define IRQ_RIOINT8          (IRQ_CPINTC0_START + 120)
#define IRQ_RIOINT9          (IRQ_CPINTC0_START + 121)
#define IRQ_RIOINT10         (IRQ_CPINTC0_START + 122)
#define IRQ_RIOINT11         (IRQ_CPINTC0_START + 123)
#define IRQ_RIOINT12         (IRQ_CPINTC0_START + 124)
#define IRQ_RIOINT13         (IRQ_CPINTC0_START + 125)
#define IRQ_RIOINT14         (IRQ_CPINTC0_START + 126)
#define IRQ_RIOINT15         (IRQ_CPINTC0_START + 127)
#define IRQ_AIFINTD          (IRQ_CPINTC0_START + 133)
#define IRQ_QMPEND22         (IRQ_CPINTC0_START + 134) /* QM */
#define IRQ_QMPEND23         (IRQ_CPINTC0_START + 135)
#define IRQ_QMPEND24         (IRQ_CPINTC0_START + 136)
#define IRQ_QMPEND25         (IRQ_CPINTC0_START + 137)
#define IRQ_QMPEND26         (IRQ_CPINTC0_START + 138)
#define IRQ_QMPEND27         (IRQ_CPINTC0_START + 139)
#define IRQ_QMPEND28         (IRQ_CPINTC0_START + 140)
#define IRQ_QMPEND29         (IRQ_CPINTC0_START + 141)
#define IRQ_QMPEND30         (IRQ_CPINTC0_START + 142)
#define IRQ_VCP0             (IRQ_CPINTC0_START + 143) /* VCP */
#define IRQ_VCP1             (IRQ_CPINTC0_START + 144)
#define IRQ_VCP2             (IRQ_CPINTC0_START + 145)
#define IRQ_VCP3             (IRQ_CPINTC0_START + 146)
#define IRQ_VCP0REVT         (IRQ_CPINTC0_START + 147)
#define IRQ_VCP0XEVT         (IRQ_CPINTC0_START + 148)
#define IRQ_VCP1REVT         (IRQ_CPINTC0_START + 149)
#define IRQ_VCP1XEVT         (IRQ_CPINTC0_START + 150)
#define IRQ_VCP2REVT         (IRQ_CPINTC0_START + 151)
#define IRQ_VCP2XEVT         (IRQ_CPINTC0_START + 152)
#define IRQ_VCP3REVT         (IRQ_CPINTC0_START + 153)
#define IRQ_VCP3XEVT         (IRQ_CPINTC0_START + 154)
#define IRQ_TCP3DA           (IRQ_CPINTC0_START + 155) /* TCP3 */
#define IRQ_TCP3DB           (IRQ_CPINTC0_START + 156)
#define IRQ_TCP3DAREVT0      (IRQ_CPINTC0_START + 157)
#define IRQ_TCP3DAREVT1      (IRQ_CPINTC0_START + 158)
#define IRQ_TCP3E            (IRQ_CPINTC0_START + 159)
#define IRQ_TCP3EREVT        (IRQ_CPINTC0_START + 160)
#define IRQ_TCP3EWEVT        (IRQ_CPINTC0_START + 161)
#define IRQ_TCP3DBREVT0      (IRQ_CPINTC0_START + 162)
#define IRQ_TCP3DBREVT1      (IRQ_CPINTC0_START + 163)
#define IRQ_UART             (IRQ_CPINTC0_START + 164) /* UART */
#define IRQ_UARTREVT         (IRQ_CPINTC0_START + 165)
#define IRQ_UARTXEVT         (IRQ_CPINTC0_START + 166)
#define IRQ_MSMCMPF4         (IRQ_CPINTC0_START + 170) /* MSMC */
#define IRQ_MSMCMPF5         (IRQ_CPINTC0_START + 171)
#define IRQ_MSMCMPF6         (IRQ_CPINTC0_START + 172)
#define IRQ_MSMCMPF7         (IRQ_CPINTC0_START + 173)
#define IRQ_QMPEND31         (IRQ_CPINTC0_START + 175) /* QM */
#define IRQ_QMCDMA0          (IRQ_CPINTC0_START + 176) /* CDMA */
#define IRQ_QMCDMA1          (IRQ_CPINTC0_START + 177)
#define IRQ_RIOCDMA0         (IRQ_CPINTC0_START + 178)
#define IRQ_PASSCDMA0        (IRQ_CPINTC0_START + 179)
#define IRQ_SMARTREFLEX0     (IRQ_CPINTC0_START + 181) /* SmartReflex sensors */
#define IRQ_SMARTREFLEX1     (IRQ_CPINTC0_START + 182)
#define IRQ_SMARTREFLEX2     (IRQ_CPINTC0_START + 183)
#define IRQ_SMARTREFLEX3     (IRQ_CPINTC0_START + 184)
#define IRQ_VPNOSMPSACK      (IRQ_CPINTC0_START + 185) /* VP */
#define IRQ_VPEQVALUE        (IRQ_CPINTC0_START + 186)
#define IRQ_VPMAXVDD         (IRQ_CPINTC0_START + 187)
#define IRQ_VPMINVDD         (IRQ_CPINTC0_START + 188)
#define IRQ_VPINIDLE         (IRQ_CPINTC0_START + 189)
#define IRQ_VPOPPCDONE       (IRQ_CPINTC0_START + 190)
#define IRQ_FFTCAINT0        (IRQ_CPINTC0_START + 192) /* FFTC */
#define IRQ_FFTCAINT1        (IRQ_CPINTC0_START + 193)
#define IRQ_FFTCAINT2        (IRQ_CPINTC0_START + 194)
#define IRQ_FFTCAINT3        (IRQ_CPINTC0_START + 195)
#define IRQ_FFTCBINT0        (IRQ_CPINTC0_START + 196)
#define IRQ_FFTCBINT1        (IRQ_CPINTC0_START + 197)
#define IRQ_FFTCBINT2        (IRQ_CPINTC0_START + 198)
#define IRQ_FFTCBINT3        (IRQ_CPINTC0_START + 199)

#define IRQ_CPINTC0_MAPLEN   8              /* number of (usable) events from CP_INTC0 */

#define NR_CPINTC0_IRQS      200            /* number of source events */
#define NR_CPINTC0_COMBINERS 8              /* number of combiners */
#define NR_CPINTC0_CHANNELS  152            /* number of output channels */
#define NR_CPINTC0_OUTPUTS   18             /* number of output host interrupts */

#define NR_CPINTC_IRQS       (NR_CPINTC0_IRQS) /* We only use INTC0 */
#define NR_CPINTC_COMBINERS  (NR_CPINTC0_COMBINERS)
#define NR_SOC_IRQS	     (IRQ_CPINTC0_START + NR_CPINTC0_IRQS)
#define NR_SOC_COMBINERS     (NR_CPINTC_COMBINERS) /* Number of combiners */

extern int cpintc_irq(unsigned int irq);
extern int cpintc_combined_irq(unsigned int irq);

/* 
 * When handling IRQs through either INTC or CP_INTC, always ack before handling irq.
 */
#define IRQ_SOC_COMBINER_PRE_ACK(irq)  1

/*
 * This macro returns 1 if the irq number is a CP_INTC interrupt at the GEM INTC level
 */
#define IRQ_SOC_COMBINER(irq)          cpintc_irq(irq)

/*
 * This macro returns 1 if the irq number is a SoC combiner combined interrupt
 */
#define IRQ_SOC_COMBINER_COMBINED(irq) cpintc_combined_irq(irq)

/*
 * This macro returns the combiner index from an host irq
 * Here it is computed based on INTC0OUT(64 + i + 10 * n) for irq 22 to 31
 */
#define IRQ_SOC_HOST_IRQ_TO_IDX(irq)   ((irq) - 64 - (get_coreid() * 10))

/*
 * This macro returns the channel number from a combiner index
 * Here it is computed based on INTC0OUT(64 + i + 10 * n) for irq 22 to 31
 */
#define IRQ_SOC_IDX_TO_CHAN(i)         ((i) + 64 + (get_coreid() * 10))

#endif /* __MACH_IRQ_C6670_H */
