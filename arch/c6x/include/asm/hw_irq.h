/* C64x Hardware interrupt definitions
 *
 * Copyright (C) 2010 Texas Instruments Incorporated
 * Written by Mark Salter (msalter@redhat.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public Licence
 * as published by the Free Software Foundation; either version
 * 2 of the Licence, or (at your option) any later version.
 */
#ifndef _ASM_HW_IRQ_H
#define _ASM_HW_IRQ_H

void set_irq_flags(unsigned int irq, unsigned int flags);

#define IRQF_VALID		(1 << 0)
#define IRQF_PROBE		(1 << 1)
#define IRQF_NOAUTOEN		(1 << 2)

#endif /* _ASM_HW_IRQ_H */
