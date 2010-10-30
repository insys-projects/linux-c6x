/*
 *  linux/arch/c6x/include/asm/rio.h
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

#ifndef _ASM_RIO_H
#define _ASM_RIO_H

#include <mach/rio.h>

/*
 * Maximum message size fo RIONET 
 */
#define MACH_RIO_MAX_MSG_SIZE            1552

/*
 * Per board RIO devices controller configuration
 */
struct tci648x_rio_board_controller_info {
        u16      ports; /* bitfield of port(s) to probe on this controller */
        u16      mode;  /* hw mode */
        u16      id;    /* host id */
        u16      init;  /* initialisation method */
        u16      size;  /* RapidIO common transport system size.
			 * 0 - Small size. 256 devices.
			 * 1 - Large size, 65536 devices.
			 */
};

/*
 * LSU registers
 */
struct tci648x_rio_lsu_reg {
        u32      reg[6];
};

#endif /* _ASM_RIO_H */
