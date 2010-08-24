/*
 *  linux/include/asm-c6x/dma.h
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
#ifndef __ASM_C6X_DMA_H
#define __ASM_C6X_DMA_H

#define MAX_DMA_ADDRESS  0xFFFFFFFF
#define MAX_DMA_CHANNELS 64

/* Reserve a DMA channel */
extern int request_dma(unsigned int dmanr, const char * device_id); 

/* Release it again */
extern void free_dma(unsigned int dmanr);

#endif /* __ASM_C6X_DMA_H */
