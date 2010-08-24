/*
 *  linux/include/asm-c6x/kmap_types.h
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Nicolas Videau (nicolas.videau@jaluna.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __ASM_C6X_KMAP_TYPES_H
#define __ASM_C6X_KMAP_TYPES_H

enum km_type {
	KM_BOUNCE_READ,
	KM_SKB_SUNRPC_DATA,
	KM_SKB_DATA_SOFTIRQ,
	KM_USER0,
	KM_USER1,
	KM_BIO_SRC_IRQ,
	KM_BIO_DST_IRQ,
	KM_PTE0,
	KM_PTE1,
	KM_IRQ0,
	KM_IRQ1,
	KM_SOFTIRQ0,
	KM_SOFTIRQ1,
	KM_TYPE_NR
};

#endif
