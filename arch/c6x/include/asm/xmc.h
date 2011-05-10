/*
 *  linux/arch/c6x/include/asm/xmc.h
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
#ifndef __ASM_C6X_XMC_H_
#define __ASM_C6X_XMC_H_

#include <asm/system.h>

/*
 * KeyStone XMC Controller
 */
#define XMC_BASE                     0x08000000
#define XMC_MPAX_BASE                0x08000000
#define XMC_XMPFAR                   0x08000200
#define XMC_XMPFSR                   0x08000204
#define XMC_XMPFCR                   0x08000208
#define XMC_MDMAARBX                 0x08000280
#define XMC_XPFCMD                   0x08000300
#define XMC_XPFACS                   0x08000304
#define XMC_XPFAC0                   0x08000310
#define XMC_XPFADDR0                 0x08000400

/*
 * MPAX segment sizes
 */
#define XMC_SEG_SIZE_DISABLED        0x00
#define XMC_SEG_SIZE_4KB             0x0b
#define XMC_SEG_SIZE_8KB             0x0c
#define XMC_SEG_SIZE_16KB            0x0d
#define XMC_SEG_SIZE_32KB            0x0e
#define XMC_SEG_SIZE_64KB            0x0f
#define XMC_SEG_SIZE_128KB           0x10
#define XMC_SEG_SIZE_256KB           0x11
#define XMC_SEG_SIZE_512KB           0x12
#define XMC_SEG_SIZE_1MB             0x13
#define XMC_SEG_SIZE_2MB             0x14
#define XMC_SEG_SIZE_4MB             0x15
#define XMC_SEG_SIZE_8MB             0x16
#define XMC_SEG_SIZE_16MB            0x17
#define XMC_SEG_SIZE_32MB            0x18
#define XMC_SEG_SIZE_64MB            0x19
#define XMC_SEG_SIZE_128MB           0x1a
#define XMC_SEG_SIZE_256MB           0x1b
#define XMC_SEG_SIZE_512MB           0x1c
#define XMC_SEG_SIZE_1GB             0x1d
#define XMC_SEG_SIZE_2GB             0x1e
#define XMC_SEG_SIZE_4GB             0x1f

/*
 * MPAX permissions
 */
#define XMC_PERM_UX                  (1 << 0)
#define XMC_PERM_UW                  (1 << 1)
#define XMC_PERM_UR                  (1 << 2)
#define XMC_PERM_SX                  (1 << 3)
#define XMC_PERM_SW                  (1 << 4)
#define XMC_PERM_SR                  (1 << 5)

/*
 * MPAX address masks
 */
#define XMC_R_ADDR_MASK              0xffffff00
#define XMC_B_ADDR_MASK              0xfffff000
#define XMC_R_ADDR_SHIFT             4
 
/*
 * XMC Prefetch buffer management
 */ 
#ifdef ARCH_HAS_XMC_PREFETCHW
static inline void xmc_prefetch_buffer_invalidate(void)
{
	*((volatile unsigned int *) (XMC_XPFCMD)) = 1;
	mfence();
}
#else
static inline void xmc_prefetch_buffer_invalidate(void) {}
#endif

/*
 * XMC Memory Protection and Address eXtension (MPAX) management
 */
#ifdef ARCH_HAS_XMC_MPAX 
/*
 * Map a system_addr address (36bit) to a CorePac native base_addr address (32bit)
 */ 
static inline int xmc_map_region(int segment, 
				 u32 base_addr,
				 u64 system_addr,
				 int size, 
				 int perm)
{
	volatile unsigned int *regl =
		(volatile unsigned int *) (XMC_MPAX_BASE + (segment << 3));
	volatile unsigned int *regh =
		(volatile unsigned int *) (XMC_MPAX_BASE + (segment << 3) + 4);

	u32 r_addr = (u32) (system_addr >> XMC_R_ADDR_SHIFT);

	if ((segment < 0) || (segment > 15))
		return -1;

	*regl = (r_addr & XMC_R_ADDR_MASK) | (perm & 0xff);
	*regh = (base_addr & XMC_B_ADDR_MASK) | (size & 0x1f);

	return 0;
}
#endif

#endif /* __ASM_C6X_XMC_H_ */
