/*
 *  linux/arch/c6x/platforms/include/mach/irq.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2010, 2011 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __MACH_C6X_IRQ_H
#define __MACH_C6X_IRQ_H

#if defined(CONFIG_SOC_TMS320C6455) || defined(CONFIG_SOC_TMS320C6457)
#include <mach/irq-c645x.h>
#elif defined(CONFIG_SOC_TMS320C6472)
#include <mach/irq-c6472.h>
#elif defined(CONFIG_SOC_TMS320C6474)
#include <mach/irq-c6474.h>
#elif defined(CONFIG_SOC_TMS320C6670)
#include <mach/irq-c6670.h>
#elif defined(CONFIG_SOC_TMS320C6678)
#include <mach/irq-c6678.h>
#else
#error "No machine IRQ definitions"
#endif

#ifdef __KERNEL__

#define IC_EVTFLAG	__SYSREGA(0x01800000, uint32_t)
#define IC_EVTSET	__SYSREGA(0x01800020, uint32_t)
#define IC_EVTCLR	__SYSREGA(0x01800040, uint32_t)
#define IC_EVTMASK	__SYSREGA(0x01800080, uint32_t)
#define IC_MEVTFLAG	__SYSREGA(0x018000a0, uint32_t)
#define IC_EXPMASK	__SYSREGA(0x018000c0, uint32_t)
#define IC_MEXPMASK	__SYSREGA(0x018000e0, uint32_t)
#define IC_INTMUX	__SYSREGA(0x01800100, uint32_t)
#define IC_AEGMUX	__SYSREGA(0x01800140, uint32_t)
#define IC_INTXSTAT	__SYSREG(0x01800180, uint32_t)
#define IC_INTXCLR	__SYSREG(0x01800184, uint32_t)
#define IC_INTDMASK	__SYSREG(0x01800188, uint32_t)
#define IC_EVTASRT	__SYSREG(0x018001c0, uint32_t)

/*
 * EVTASRT bits
 */
#define IC_B_EVTPULSE4   (1 << 4)

#define assert_event(evt) (IC_EVTASRT = (evt))

#define NR_BOARD_IRQS    0 /* Not used */

#endif /* __KERNEL__ */
#endif /* __MACH_C6X_IRQ_H */
