/*
 *  linux/include/asm-c6x/cp-intc.h
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
#ifndef __ASM_C6X_CP_INTC_H_
#define __ASM_C6X_CP_INTC_H_

/*
 * CP_INTC(0-2) register layout
 */
#define CPINTC_REG_BASE(n)          (0x02600000 + (0x4000 * (n)))

#define CPINTC_REV(n)	             __SYSREGA(CPINTC_REG_BASE(n) + 0x0000, uint32_t)
#define CPINTC_CTRL(n)	             __SYSREGA(CPINTC_REG_BASE(n) + 0x0004, uint32_t)
#define CPINTC_HOSTCTRL(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x000c, uint32_t)
#define CPINTC_GLOBALHINTEN(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0010, uint32_t)
#define CPINTC_GLOBALNESTLEVEL(n)    __SYSREGA(CPINTC_REG_BASE(n) + 0x001c, uint32_t)
#define CPINTC_STATUSIDXSET(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0020, uint32_t)
#define CPINTC_STATUSIDXCLR(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0024, uint32_t)
#define CPINTC_ENABLEIDXSET(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0028, uint32_t)
#define CPINTC_ENABLEIDXCLR(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x002c, uint32_t)
#define CPINTC_HINTIDXSET(n) 	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0034, uint32_t)
#define CPINTC_HINTIDXCLR(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0038, uint32_t)
#define CPINTC_VECTORNULL(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0058, uint32_t)
#define CPINTC_GLOBALPRIOINT(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0080, uint32_t)
#define CPINTC_GLOBALVECTORADDR(n)   __SYSREGA(CPINTC_REG_BASE(n) + 0x0080, uint32_t)
#define CPINTC_RAWSTATUS(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0200, uint32_t)
#define CPINTC_ENASTATUS(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0280, uint32_t)
#define CPINTC_ENABLE(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0300, uint32_t)
#define CPINTC_ENABLECLR(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0380, uint32_t)
#define CPINTC_CHMAP(n)	             __SYSREGA(CPINTC_REG_BASE(n) + 0x0400, uint32_t)
#define CPINTC_HINTMAP(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0800, uint32_t)
#define CPINTC_HINT(n)	             __SYSREGA(CPINTC_REG_BASE(n) + 0x0900, uint32_t)
#define CPINTC_POLARITY(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0d00, uint32_t)
#define CPINTC_TYPE(n)      	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0d80, uint32_t)
#define CPINTC_DBGSELECT(n)    	     __SYSREGA(CPINTC_REG_BASE(n) + 0x0f00, uint32_t)
#define CPINTC_SECUREENABLE(n) 	     __SYSREGA(CPINTC_REG_BASE(n) + 0x1000, uint32_t)
#define CPINTC_SECUREENABLECLR(n)    __SYSREGA(CPINTC_REG_BASE(n) + 0x1080, uint32_t)
#define CPINTC_NESTLEVEL(n)          __SYSREGA(CPINTC_REG_BASE(n) + 0x1100, uint32_t)
#define CPINTC_ENABLEHINT(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x1500, uint32_t)
#define CPINTC_VECTORADDR(n)	     __SYSREGA(CPINTC_REG_BASE(n) + 0x1f00, uint32_t)

#define CPINTC_NO_NESTING            0x0
#define CPINTC_AUTOMATIC_GLB_NESTING 0x1
#define CPINTC_AUTOMATIC_IND_NESTING 0x2
#define CPINTC_MANUAL_NESTING        0x3

#endif /* __ASM_C6X_CP_INTC_H_ */
