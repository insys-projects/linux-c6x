/*
 *  linux/include/asm-c6x/irq.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_IRQ_H_
#define __ASM_C6X_IRQ_H_

#include <asm/hardware.h>
#include <mach/irq.h>

#define irq_canonicalize(irq)  (irq)

/*
 * Number of C6x interrupts:
 * 16 processor interrupt sources
 * 32 platform interrupt sources
 * 16 CIC output events
 */
#define SYS_IRQS            16
#define NR_IRQS             32
#define CIC_IRQS            16

/*
 * Processor interrupt definitions
 * General interrupt sources are the level 1-15.
 */
#define INT0                0    /* RESET */
#define INT1		    1	 /* NMI */
#define INT2		    2	 /* Reserved */
#define INT3		    3	 /* Reserved */
#define INT4		    4	 /* level 4 interrupt */
#define INT5		    5	 /* level 5 interrupt */
#define INT6		    6	 /* level 6 interrupt */
#define INT7		    7	 /* level 7 interrupt */
#define INT8		    8	 /* level 8 interrupt */
#define INT9		    9	 /* level 9 interrupt */
#define INT10		    10   /* level 10 interrupt */
#define INT11		    11   /* level 11 interrupt */
#define INT12		    12   /* level 12 interrupt */
#define INT13		    13   /* level 13 interrupt */
#define INT14		    14   /* level 14 interrupt */
#define INT15		    15   /* level 15 interrupt */

/*
 * CIC output events
 */
#define CIC0                0
#define CIC1		    1
#define CIC2		    2
#define CIC3		    3
#define CIC4		    4
#define CIC5		    5
#define CIC6		    6
#define CIC7		    7
#define CIC8		    8
#define CIC9		    9
#define CIC10		    10
#define CIC11		    11
#define CIC12		    12
#define CIC13		    13
#define CIC14		    14
#define CIC15		    15

extern void irq_map(unsigned int irq_src, unsigned int cpu_irq);

#endif /* __ASM_C6X_IRQ_H_ */
