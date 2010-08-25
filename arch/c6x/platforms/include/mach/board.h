/*
 *  linux/arch/c6x/platforms/mach/board.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_BOARD_H
#define __ASM_C6X_BOARD_H

#ifdef CONFIG_ARCH_BOARD_DSK6455
#include <mach/board-dsk6455.h>
#endif
#ifdef CONFIG_ARCH_BOARD_EVM6472
#include <mach/board-evm6472.h>
#endif
#ifdef CONFIG_ARCH_BOARD_EVM6474
#include <mach/board-evm6474.h>
#endif

#endif
