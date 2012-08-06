/*
 *  linux/arch/c6x/platforms/include/mach/cache.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2007, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@virtuallogix.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

/* 
 * Cache line size
 */
#define L1D_CACHE_BYTES   64
#define L1P_CACHE_BYTES   32
#define L2_CACHE_BYTES    128

#define L1_CACHE_SHIFT    6

/*
 * L2 used as cache
 */
#define L2MODE_SIZE       L2MODE_MAX_CACHE  /* Use all possible L2 as cache */

/*
 * Internal Memory Control Registers for caches
 */
#define IMCR_CCFG         0x01840000  
#define IMCR_L1PCFG       0x01840020
#define IMCR_L1PCC        0x01840024
#define IMCR_L1DCFG       0x01840040
#define IMCR_L1DCC        0x01840044
#define IMCR_L2ALLOC0 	  0x01842000
#define IMCR_L2ALLOC1 	  0x01842004
#define IMCR_L2ALLOC2 	  0x01842008
#define IMCR_L2ALLOC3 	  0x0184200c
#define IMCR_L2WBAR 	  0x01844000
#define IMCR_L2WWC 	  0x01844004
#define IMCR_L2WIBAR 	  0x01844010
#define IMCR_L2WIWC 	  0x01844014
#define IMCR_L2IBAR 	  0x01844018
#define IMCR_L2IWC 	  0x0184401c
#define IMCR_L1PIBAR	  0x01844020
#define IMCR_L1PIWC 	  0x01844024
#define IMCR_L1DWIBAR 	  0x01844030
#define IMCR_L1DWIWC 	  0x01844034
#define IMCR_L1DWBAR 	  0x01844040
#define IMCR_L1DWWC 	  0x01844044
#define IMCR_L1DIBAR	  0x01844048
#define IMCR_L1DIWC 	  0x0184404c
#define IMCR_L2WB 	  0x01845000
#define IMCR_L2WBINV 	  0x01845004
#define IMCR_L2INV 	  0x01845008
#define IMCR_L1PINV 	  0x01845028
#define IMCR_L1DWB        0x01845040
#define IMCR_L1DWBINV     0x01845044
#define IMCR_L1DINV       0x01845048
#define IMCR_MAR_BASE 	  0x01848000
#define IMCR_MAR96_111 	  0x01848180
#define IMCR_MAR128_191   0x01848200
#define IMCR_MAR224_239   0x01848380
#define IMCR_L2MPFAR      0x0184a000
#define IMCR_L2MPFSR      0x0184a004
#define IMCR_L2MPFCR      0x0184a008
#define IMCR_L2MPLK0      0x0184a100
#define IMCR_L2MPLK1      0x0184a104
#define IMCR_L2MPLK2      0x0184a108
#define IMCR_L2MPLK3      0x0184a10c
#define IMCR_L2MPLKCMD    0x0184a110
#define IMCR_L2MPLKSTAT   0x0184a114
#define IMCR_L2MPPA_BASE  0x0184a200
#define IMCR_L1PMPFAR     0x0184a400
#define IMCR_L1PMPFSR     0x0184a404
#define IMCR_L1PMPFCR     0x0184a408
#define IMCR_L1PMPLK0     0x0184a500
#define IMCR_L1PMPLK1     0x0184a504
#define IMCR_L1PMPLK2     0x0184a508
#define IMCR_L1PMPLK3     0x0184a50c
#define IMCR_L1PMPLKCMD   0x0184a510
#define IMCR_L1PMPLKSTAT  0x0184a514
#define IMCR_L1PMPPA_BASE 0x0184a600
#define IMCR_L1DMPFAR     0x0184ac00
#define IMCR_L1DMPFSR     0x0184ac04
#define IMCR_L1DMPFCR     0x0184ac08
#define IMCR_L1DMPLK0     0x0184ad00
#define IMCR_L1DMPLK1     0x0184ad04
#define IMCR_L1DMPLK2     0x0184ad08
#define IMCR_L1DMPLK3     0x0184ad0c
#define IMCR_L1DMPLKCMD   0x0184ad10
#define IMCR_L1DMPLKSTAT  0x0184ad14
#define IMCR_L1DMPPA_BASE 0x0184ae00
#define IMCR_L2PDWAKE0    0x0184c040
#define IMCR_L2PDWAKE1    0x0184c044
#define IMCR_L2PDSLEEP0   0x0184c050
#define IMCR_L2PDSLEEP1   0x0184c054
#define IMCR_L2PDSTAT0    0x0184c060
#define IMCR_L2PDSTAT1    0x0184c064

/* 
 * MAR register bits
 */
#define IMCR_MAR_PC       (1 << 0)
#define IMCR_MAR_PFX      (1 << 3)

/* 
 * Physical memory granularity for MAR registers 
 */
#define IMCR_MAR_SIZE     0x01000000
