/*
 *  linux/arch/c6x/platforms/mach/nand-gpio.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_NAND_GPIO_H
#define __ASM_C6X_NAND_GPIO_H

#if defined(CONFIG_ARCH_BOARD_EVM6457) || defined(CONFIG_ARCH_BOARD_EVM6472) || defined(CONFIG_ARCH_BOARD_EVM6474L)
/*
 * C647x/C645x LC EVM definitions
 */ 
#define GPIO_NAND_CLE	    8
#define GPIO_NAND_ALE	    9
#define GPIO_NAND_NWE	    10
#define GPIO_NAND_RDY	    11
#define GPIO_NAND_NRE	    12
#define GPIO_NAND_NCE	    13
#define GPIO_NAND_NWP	    14

#define GPIO_NAND_DATA_MASK 0xff

#define xdelay()            ndelay(20)

#define GPIO_NAND_USE_READY_BUSY_PIN
#define GPIO_NAND_USE_WRITE_PROTECT
#endif /* defined(CONFIG_ARCH_BOARD_EVM6457) || defined(CONFIG_ARCH_BOARD_EVM6472) || defined(CONFIG_ARCH_BOARD_EVM6474L) */

#if defined(CONFIG_ARCH_BOARD_EVM6474)
/*
 * Custom EVM6488/EVM6474 definitions
 */ 
#define GPIO_NAND_CLE	    8
#define GPIO_NAND_ALE	    9
#define GPIO_NAND_NWE	    12
#define GPIO_NAND_RDY	    13
#define GPIO_NAND_NRE	    15
#define GPIO_NAND_NCE	    10
#undef  GPIO_NAND_NWP       /* not used */

#define GPIO_NAND_DATA_MASK 0xff

#define xdelay()            ndelay(15)

#define GPIO_NAND_USE_READY_BUSY_PIN
#undef  GPIO_NAND_USE_WRITE_PROTECT
#endif /* defined(CONFIG_ARCH_BOARD_EVM6474) */

#if defined(CONFIG_ARCH_BOARD_EVMTCI6616)
/*
 * EVM TCI6616 EVM definitions
 */ 
#define GPIO_NAND_CLE	    8
#define GPIO_NAND_ALE	    9
#define GPIO_NAND_NWE	    12
#define GPIO_NAND_RDY	    13
#define GPIO_NAND_NRE	    15
#define GPIO_NAND_NCE	    14
#undef  GPIO_NAND_NWP       /* not used */

#define GPIO_NAND_DATA_MASK 0xff

#define xdelay()            ndelay(15)

#define GPIO_NAND_USE_READY_BUSY_PIN
#undef  GPIO_NAND_USE_WRITE_PROTECT
#endif /* defined(CONFIG_ARCH_BOARD_EVMTCI6616) */

#if defined(CONFIG_ARCH_BOARD_EVM6670)
/*
 * EVM C6670 LC EVM definitions
 */ 
#define GPIO_NAND_CLE	    8
#define GPIO_NAND_ALE	    9
#define GPIO_NAND_NWE	    10
#define GPIO_NAND_RDY	    11
#define GPIO_NAND_NRE	    12
#define GPIO_NAND_NCE	    13
#undef  GPIO_NAND_NWP       /* not used */

#define GPIO_NAND_DATA_MASK 0xff

#define xdelay()            ndelay(15)

#define GPIO_NAND_USE_READY_BUSY_PIN
#undef  GPIO_NAND_USE_WRITE_PROTECT
#endif /* defined(CONFIG_ARCH_BOARD_EVM6670) */

#endif /* __ASM_C6X_NAND_GPIO_H */
