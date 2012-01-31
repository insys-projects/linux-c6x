/*
 *  linux/arch/c6x/platforms/include/mach/rio.h
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
#ifndef _MACH_RIO_H
#define _MACH_RIO_H
#ifdef CONFIG_RAPIDIO_TCI648X
#include <mach/rio-tci648x.h>
#elif CONFIG_TI_KEYSTONE_RAPIDIO
#if defined(CONFIG_SOC_TMS320C6670) || defined(CONFIG_SOC_TMS320C6678)
#include <mach/rio-c667x.h>
#endif
#else
#error "No machine RapidIO definitions"
#endif
#endif /* _MACH_RIO_H */
