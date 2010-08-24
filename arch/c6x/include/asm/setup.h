/*
 *  linux/include/asm-c6x/setup.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_SETUP_H
#define __ASM_C6X_SETUP_H

#include <asm/hardware.h>

#define CL_SIZE	            (500)
#define COMMAND_LINE_SIZE   CL_SIZE

/* Internal, cat on the end of kernel, or separate fixed address romfs. */
#define FIXED_ROMARRAY      0x80400000
#define FIXED_ROMARRAY_SIZE 0x00300000

/* Where paged memory can start */
#define RAM_START           (FIXED_ROMARRAY + FIXED_ROMARRAY_SIZE)

#endif /* __ASM_C6X_SETUP_H */
