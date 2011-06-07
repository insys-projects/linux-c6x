/*
 *  linux/arch/c6x/platforms/include/mach/hardware.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010, 2011 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *          Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_MACH_HARDWARE_H
#define __ASM_C6X_MACH_HARDWARE_H

/*
 * Misc. hardware defines which should probably have their own home
 * so we can get rid of hardware.h altogether.
 */

/* 
 * Memory address space
 */
#if defined(CONFIG_SOC_TMS320C6455)
#define RAM_EMIFA_CE2     0xa0000000
#define RAM_EMIFA_CE3     0xb0000000
#define RAM_EMIFA_CE4     0xc0000000
#define RAM_EMIFA_CE5     0xd0000000
#define RAM_DDR2_CE0      0xe0000000
#define RAM_MEMORY_START  RAM_DDR2_CE0
#elif defined(CONFIG_SOC_TMS320C6457)
#define RAM_EMIFA_CE2     0xa0000000
#define RAM_EMIFA_CE3     0xb0000000
#define RAM_EMIFA_CE4     0xc0000000
#define RAM_EMIFA_CE5     0xd0000000
#define RAM_DDR2_CE0      0xe0000000
#define RAM_MEMORY_START  RAM_DDR2_CE0
#elif defined(CONFIG_SOC_TMS320C6472)
#define RAM_SRAM          0x00800000
#define RAM_SRAM_BASE     0x10800000
#define RAM_SRAM_OFFSET   0x01000000
#define RAM_SRAM_SIZE     0x00080000
#define RAM_DDR2_CE0      0xe0000000
#define RAM_MEMORY_START  RAM_DDR2_CE0
#elif defined(CONFIG_SOC_TMS320C6474)
#define RAM_SRAM          0x00800000
#define RAM_SRAM_BASE     0x10800000
#define RAM_SRAM_OFFSET   0x01000000
#define RAM_SRAM_SIZE     0x00100000
#define RAM_DDR2_CE0      0x80000000
#define RAM_MEMORY_START  RAM_DDR2_CE0
#elif defined(CONFIG_SOC_TMS320C6670)
#define RAM_SRAM          0x00800000
#define RAM_SRAM_BASE     0x10800000
#define RAM_SRAM_OFFSET   0x01000000
#define RAM_SRAM_SIZE     0x00100000
#define RAM_DDR2_CE0      0x80000000
#define RAM_MSM_BASE      0x0c000000
#define RAM_MSM_SIZE      0x00200000
#define RAM_MSM_CO_BASE   0x2c000000
#define RAM_MEMORY_START  RAM_DDR2_CE0
#elif defined(CONFIG_SOC_TMS320C6678)
#define RAM_SRAM          0x00800000
#define RAM_SRAM_BASE     0x10800000
#define RAM_SRAM_OFFSET   0x01000000
#define RAM_SRAM_SIZE     0x00080000
#define RAM_EMIFA_CE2     0x70000000
#define RAM_EMIFA_CE3     0x74000000
#define RAM_EMIFA_CE4     0x78000000
#define RAM_EMIFA_CE5     0x7c000000
#define RAM_DDR2_CE0      0x80000000
#define RAM_MSM_BASE      0x0c000000
#define RAM_MSM_SIZE      0x00400000
#define RAM_MSM_CO_BASE   0x2c000000
#define RAM_MEMORY_START  RAM_DDR2_CE0
#else
#error "No SoC memory address space defines"
#endif

/* 
 * VBUS clock Rate in MHz (1-255)
 */
#define VBUSCLK           165

/*
 * PSC Registers
 */
#if defined(CONFIG_SOC_TMS320C6474)
#define PSC_PTCMD                    0x02ac0120
#define PSC_PDSTAT0                  0x02ac0200
#define PSC_PDSTAT1                  0x02ac0204
#define PSC_PDSTAT2                  0x02ac0208
#define PSC_PDSTAT3                  0x02ac020c
#define PSC_PDSTAT4                  0x02ac0210
#define PSC_PDSTAT5                  0x02ac0214
#define PSC_PDCTL0                   0x02ac0300
#define PSC_PDCTL1                   0x02ac0304
#define PSC_PDCTL2                   0x02ac0308
#define PSC_PDCTL3                   0x02ac030c
#define PSC_PDCTL4                   0x02ac0310
#define PSC_PDCTL5                   0x02ac0314
#define PSC_MDSTAT0                  0x02ac0800
#define PSC_MDSTAT1                  0x02ac0804
#define PSC_MDSTAT2                  0x02ac0808
#define PSC_MDSTAT3                  0x02ac080c
#define PSC_MDSTAT4                  0x02ac0810
#define PSC_MDSTAT5                  0x02ac0814
#define PSC_MDSTAT6                  0x02ac0818
#define PSC_MDSTAT7                  0x02ac081c
#define PSC_MDSTAT8                  0x02ac0820
#define PSC_MDSTAT9                  0x02ac0824
#define PSC_MDSTAT10                 0x02ac0828
#define PSC_MDCTL0                   0x02ac0a00
#define PSC_MDCTL1                   0x02ac0a04
#define PSC_MDCTL2                   0x02ac0a08
#define PSC_MDCTL3                   0x02ac0a0c
#define PSC_MDCTL4                   0x02ac0a10
#define PSC_MDCTL5                   0x02ac0a14
#define PSC_MDCTL6                   0x02ac0a18
#define PSC_MDCTL7                   0x02ac0a1c
#define PSC_MDCTL8                   0x02ac0a20
#define PSC_MDCTL9                   0x02ac0a24
#define PSC_MDCTL10                  0x02ac0a28
#define PSC_MDCTL11                  0x02ac0a2c
#endif  /* CONFIG_SOC_TMS320C6474 */

#if defined(CONFIG_SOC_TMS320C6472)
#define PSC_PID                      0x02ae0000
#define PSC_INTEVAL                  0x02ae0018
#define PSC_MERRPR                   0x02ae0040
#define PSC_MERRCR                   0x02ae0050
#define PSC_PTCMD                    0x02ae0120
#define PSC_PTSTAT                   0x02ae0128
#define PSC_MDSTAT0                  0x02ae0800
#define PSC_MDSTAT1                  0x02ae0804
#define PSC_MDSTAT2                  0x02ae0808
#define PSC_MDSTAT3                  0x02ae080c
#define PSC_MDSTAT4                  0x02ae0810
#define PSC_MDSTAT5                  0x02ae0814
#define PSC_MDSTAT6                  0x02ae0818
#define PSC_MDSTAT7                  0x02ae081c
#define PSC_MDSTAT8                  0x02ae0820
#define PSC_MDSTAT9                  0x02ae0824
#define PSC_MDSTAT10                 0x02ae0828
#define PSC_MDSTAT11                 0x02ae082c
#define PSC_MDSTAT12                 0x02ae0830
#define PSC_MDSTAT13                 0x02ae0834
#define PSC_MDCTL0                   0x02ae0a00
#define PSC_MDCTL1                   0x02ae0a04
#define PSC_MDCTL2                   0x02ae0a08
#define PSC_MDCTL3                   0x02ae0a0c
#define PSC_MDCTL4                   0x02ae0a10
#define PSC_MDCTL5                   0x02ae0a14
#define PSC_MDCTL6                   0x02ae0a18
#define PSC_MDCTL7                   0x02ae0a1c
#define PSC_MDCTL8                   0x02ae0a20
#define PSC_MDCTL9                   0x02ae0a24
#define PSC_MDCTL10                  0x02ae0a28
#define PSC_MDCTL11                  0x02ae0a2c
#define PSC_MDCTL12                  0x02ae0a30
#define PSC_MDCTL13                  0x02ae0a34
#endif  /* CONFIG_SOC_TMS320C6472 */

#if defined(CONFIG_SOC_TMS320C6670) || defined(CONFIG_SOC_TMS320C6678)
#define PSC_BASE                     0x02350000
#define PSC_PTCMD                    0x02350120
#define PSC_PTSTAT                   0x02350128
#define PSC_PDCTL0                   0x02350300
#define PSC_MDSTAT0                  0x02350800
#define PSC_MDCTL0                   0x02350a00

#ifdef CONFIG_SOC_TMS320C6678
#define PSC_EMIF25_SPI               3  /* EMIF16 and SPI */
#define PSC_CORE0_TIMER0_BASE        (PSC_MDCTL0 + (15 * 4)) /* Base of Core and Timer 0 */

/* Get the Power domain for core */
#define PD_GEM0                      8
#define GET_PD(x)                    (x + PD_GEM0) 
#define MDCTL_NEXT_STATE_DIS	     0
#define MDCTL_NEXT_STATE_EN	     3

#define MDCTL_NEXT_STATE_MASK	     0x1F
#define MDCTL_LRSTZ_MASK	     0x100

#define MDSTAT_STATE_DIS	     0
#define MDSTAT_STATE_EN		     3
#define MDSTAT_STATE_MASK	     0x1F

#endif /* CONFIG_SOC_TMS320C6678 */
#define PSC_DEBUGSS_TRC              5  /* Debug trace */
#define PSC_TETB_TRC                 6
#define PSC_PA                       7  /* PA */
#define PSC_CPGMAC                   8  /* SGMII */
#define PSC_SA                       9  /* SA */
#define PSC_PCIE                     10 /* PCIe */
#define PSC_SRIO                     11 /* sRIO */
#define PSC_HYPERLINK                12 /* HyperLink */
#define PSC_MSMCSRAM                 14 /* MSMC RAM */

#define PSC_SYNCRESET                0x1000
#define PSC_DISABLE                  0x0000
#define PSC_ENABLE                   0x0003
#endif /* CONFIG_SOC_TMS329C6670 || CONFIG_SOC_TMS320C6678 */

/*
 * TCI648x megamodules misc registers & constants
 */
#define MM_REVID                     0x01812000

#if defined(CONFIG_SOC_TMS320C6474)
#define CORE_NUM                     3
#define C6X_SOC_HAS_CORE_REV
#elif defined(CONFIG_SOC_TMS320C6472)
#define CORE_NUM                     6
#elif defined(CONFIG_SOC_TMS320C6670)
#define CORE_NUM                     4
#elif defined(CONFIG_SOC_TMS320C6678)
#define CORE_NUM                     8
#else
#define CORE_NUM                     1
#endif

/*
 * Inter-DSP Interrupt Registers
 */
#if defined(CONFIG_SOC_TMS320C6474)
#define IPCGR_BASE                   0x02880900
#define IPCAR_BASE                   0x02880940
#endif
#if defined(CONFIG_SOC_TMS320C6472)
#define NMIGR_BASE                   0x02a80500
#define IPCGR_BASE                   0x02a80540
#define IPCAR_BASE                   0x02a80580
#endif
#if defined(CONFIG_SOC_TMS320C6670) || defined(CONFIG_SOC_TMS320C6678)
#define NMIGR_BASE                   0x02620200
#define IPCGR_BASE                   0x02620240
#define IPCGRH                       0x0262027c
#define IPCAR_BASE                   0x02620280
#define IPCARH                       0x026202bc
#endif

#if defined(CONFIG_SOC_TMS320C6474)
/*
 * MCBSP registers base
 */
#define MCBSP0_BASE_ADDR             0x028c0000
#define MCBSP1_BASE_ADDR             0x028d0000
#define MCBSP0_EDMA_BASE_ADDR        0x30000000
#define MCBSP1_EDMA_BASE_ADDR        0x34000000
#endif

#if defined(CONFIG_SOC_TMS320C6670) || defined(CONFIG_SOC_TMS320C6678)
#define UART_BASE_ADDR               0x02540000
#endif

#if defined(CONFIG_SOC_TMS320C6670) || defined(CONFIG_SOC_TMS320C6678)
/*
 * KeyStone specific features
 */
#define ARCH_HAS_XMC_PREFETCHW       /* Use prefetch buffers */
#define ARCH_HAS_XMC_MPAX            /* Use XMC address extension  */
#define ARCH_HAS_MSM                 /* Architecture has MSM */
#endif /* (CONFIG_SOC_TMS320C6670) || defined(CONFIG_SOC_TMS320C6678) */

#endif  /* __ASM_C6X_MACH_HARDWARE_H */
