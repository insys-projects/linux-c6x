/*
 *  linux/arch/c6x/platforms/include/mach/emif.h
 *
 *  External Memory Interface for Texas Instruments C64x+
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __MACH_C6X_EMIF_H
#define __MACH_C6X_EMIF_H

#if defined(CONFIG_SOC_TMS320C6455) || defined(CONFIG_SOC_TMS320C6457)
#define EMIF_BASE               0x70000000
#elif defined(CONFIG_SOC_TMS320C6678)
#define EMIF_BASE               0x20C00000
#else
#error "EMIF not supported on this SoC"
#endif

#ifdef ARCH_HAS_EMIFA
#define EMIFA_MIDR	        __REG(EMIF_BASE + 0x00)
#define EMIFA_STAT	        __REG(EMIF_BASE + 0x04)
#define EMIFA_BPRIO	        __REG(EMIF_BASE + 0x20)
#define EMIFA_CE2CFG	        __REG(EMIF_BASE + 0x80)
#define EMIFA_CE3CFG	        __REG(EMIF_BASE + 0x84)
#define EMIFA_CE4CFG	        __REG(EMIF_BASE + 0x88)
#define EMIFA_CE5CFG	        __REG(EMIF_BASE + 0x8C)
#define EMIFA_AWCC	        __REG(EMIF_BASE + 0xA0)
#define EMIFA_INTRAW	        __REG(EMIF_BASE + 0xC0)
#define EMIFA_INTMSK	        __REG(EMIF_BASE + 0xC4)
#define EMIFA_INTMSKSET	        __REG(EMIF_BASE + 0xC8)
#define EMIFA_INTMSKCLR	        __REG(EMIF_BASE + 0x0C)

/* Layout of CEnCFG register for Sync mode */
#define EMIFA_CFG_SYNC		BIT(31)
#define EMIFA_CFG_RD_BE_EN	BIT(10)
#define EMIFA_CFG_CE_EXT	BIT(9)
#define EMIFA_CFG_R_ENABLE	BIT(8)
#define EMIFA_CFG_W_LTNCY(n)	(((n) & 3) << 6)
#define EMIFA_CFG_R_LTNCY(n)	(((n) & 3) << 2)
#define EMIFA_CFG_WIDTH_8	0
#define EMIFA_CFG_WIDTH_16	1
#define EMIFA_CFG_WIDTH_32	2
#define EMIFA_CFG_WIDTH_64	3

/* Layout of CEnCFG register for Async mode */
#define EMIFA_CFG_ASYNC		(0 << 31)
#define EMIFA_CFG_SS		BIT(30)
#define EMIFA_CFG_BWEM		BIT(29)
#define EMIFA_CFG_AE		BIT(28)
#define EMIFA_CFG_W_SETUP(n)	((((n) - 1) & 0x0f) << 24)
#define EMIFA_CFG_W_STROBE(n)	((((n) - 1) & 0x3f) << 18)
#define EMIFA_CFG_W_HOLD(n)	((((n) - 1) & 0x07) << 15)
#define EMIFA_CFG_R_SETUP(n)	((((n) - 1) & 0x0f) << 11)
#define EMIFA_CFG_R_STROBE(n)	((((n) - 1) & 0x3f) << 5)
#define EMIFA_CFG_R_HOLD(n)	((((n) - 1) & 0x07) << 2)
/* Bus width same as Sync mode */
#endif /* ARCH_HAS_EMIFA */

#ifdef ARCH_HAS_EMIF16
#define EMIF16_RCSR             0x00
#define EMIF16_AWCCR            0x04
#define EMIF16_A1CR             0x10
#define EMIF16_A2CR             0x14
#define EMIF16_A3CR             0x18
#define EMIF16_A4CR             0x1C
#define EMIF16_IRR              0x40
#define EMIF16_IMR              0x44
#define EMIF16_IMSR             0x48
#define EMIF16_IMCR             0x4C
#define EMIF16_NANDFCR          0x60
#define EMIF16_NANDFSR          0x64
#define EMIF16_PMCR             0x68
#define EMIF16_NFECCCS2         0x70
#define EMIF16_NFECCCS3         0x74
#define EMIF16_NFECCCS4         0x78
#define EMIF16_NFECCCS5         0x7C
#define EMIF16_NANDF4BECCLR     0xBC
#define EMIF16_NANDF4BECC1R     0xC0
#define EMIF16_NANDF4BECC2R     0xC4
#define EMIF16_NANDF4BECC3R     0xC8
#define EMIF16_NANDF4BECC4R     0xCC
#define EMIF16_NANDFEA1R        0xD0
#define EMIF16_NANDFEA2R        0xD4
#define EMIF16_NANDFEV1R        0xD8
#define EMIF16_NANDFEV2R        0xDC
#endif /* ARCH_HAS_EMIF16 */

#endif /* __MACH_C6X_EMIF_H */
