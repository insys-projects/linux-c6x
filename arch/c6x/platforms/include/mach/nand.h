/*
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated
 *  Author: Sandeep Paulraj <s-paulraj@ti.com>
 *          Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ARCH_C6X_PLATFORMS_MACH_NAND_H
#define __ARCH_C6X_PLATFORMS_MACH_NAND_H

#include <linux/mtd/nand.h>

#define NRCSR_OFFSET		        0x00
#define AWCCR_OFFSET		        0x04
#define A1CR_OFFSET		        0x10
#define A2CR_OFFSET		        0x14
#define A3CR_OFFSET		        0x18
#define A4CR_OFFSET		        0x1c
#define IRR_OFFSET		        0x40
#define IMR_OFFSET		        0x44
#define IMSR_OFFSET		        0x48
#define IMCR_OFFSET		        0x4c
#define NANDFCR_OFFSET		        0x60
#define NANDFSR_OFFSET		        0x64
#define PMCR_OFFSET		        0x68
#define NANDF1ECC_OFFSET	        0x70
#define NANDF4BECCLR_OFFSET	        0xbc
#define NANDFEA1R_OFFSET	        0xd0
#define NANDFEA2R_OFFSET	        0xd4
#define NANDFEV1R_OFFSET	        0xd8
#define NANDFEV2R_OFFSET	        0xdc

/* 4-bit ECC syndrome registers */
#define NAND_4BIT_ECC_LOAD_OFFSET	0xbc
#define NAND_4BIT_ECC1_OFFSET		0xc0
#define NAND_4BIT_ECC2_OFFSET		0xc4
#define NAND_4BIT_ECC3_OFFSET		0xc8
#define NAND_4BIT_ECC4_OFFSET		0xcc
#define NAND_ERR_ADD1_OFFSET		0xd0
#define NAND_ERR_ADD2_OFFSET		0xd4
#define NAND_ERR_ERRVAL1_OFFSET		0xd8
#define NAND_ERR_ERRVAL2_OFFSET		0xdc

/* NOTE:  boards don't need to use these address bits
 * for ALE/CLE unless they support booting from NAND.
 * They're used unless platform data overrides them.
 */
#define	MASK_ALE		        0x08
#define	MASK_CLE		        0x10

/*
 * Platform data 
 */
struct davinci_nand_pdata {		
	uint32_t		mask_ale;
	uint32_t		mask_cle;

	/* for packages using two chipselects */
	uint32_t		mask_chipsel;

	/* board's default static partition info */
	struct mtd_partition	*parts;
	unsigned		nr_parts;

	nand_ecc_modes_t	ecc_mode;
	u8			ecc_bits;

	/* e.g. NAND_BUSWIDTH_16 or NAND_USE_FLASH_BBT */
	unsigned		options;

	/* Main and mirror bbt descriptor overrides */
	struct nand_bbt_descr	*bbt_td;
	struct nand_bbt_descr	*bbt_md;
};

#ifdef CONFIG_SOC_TMS320C6678
/*
 * This is an ugly hack to initialize properly the NAND Flash timings on Keystone
 */
#define machine_is_davinci_evm()					\
	(( {								\
			davinci_nand_writel(info, A1CR_OFFSET,		\
					    (0				\
					     | (0 << 31)     /* selectStrobe */ \
					     | (0 << 30)     /* extWait (never with NAND) */ \
					     | (0xf << 26)   /* writeSetup  10 ns */ \
					     | (0x3f << 20)  /* writeStrobe 40 ns */ \
					     | (7 << 17)     /* writeHold   10 ns */ \
					     | (0xf << 13)   /* readSetup   10 ns */ \
					     | (0x3f << 7)   /* readStrobe  60 ns */ \
					     | (7 << 4)      /* readHold    10 ns */ \
					     | (3 << 2)      /* turnAround  40 ns */ \
					     | (0 << 0)));   /* asyncSize   8-bit bus */ \
			davinci_nand_writel(info, AWCCR_OFFSET,		\
					    (0x80            /* max extended wait cycle */ \
					     | (0 << 16)     /* CS2 uses WAIT0 */	\
					     | (0 << 28)));  /* WAIT0 polarity low */ \
			davinci_nand_writel(info, IRR_OFFSET,		\
					    (1		     /* clear async timeout */ \
					     | (1 << 2)));   /* clear wait rise */ \
		} ), 0)
#else
#define machine_is_davinci_evm() 0
#endif

#endif	/* __ARCH_C6X_PLATFORMS_MACH_NAND_H */
