/*
 *  linux/include/asm-c6x/resource.h
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
#ifndef __ASM_C6X_RESOURCE_H
#define __ASM_C6X_RESOURCE_H

#ifdef _STK_LIM
#undef _STK_LIM
#endif
/*
 * We do not have MMU meaning that stack is really allocated and wastes memory.
 * For example NPTL relies on this value for allocating thread stacks.
 * Thus align this value on the stack size generated in the ELF C6x
 * binaries (128KB).
 */
#define _STK_LIM  (128 * 1024)

#include <asm-generic/resource.h>

#endif /* __ASM_C6X_RESOURCE_H */
