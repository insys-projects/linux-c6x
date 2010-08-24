/*
 *  arch/c6x/include/asm/procinfo.h
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Mark Salter (msalter@redhat.com)
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_PROCINFO_H
#define __ASM_PROCINFO_H

#ifdef __KERNEL__

struct proc_info_list {
	unsigned int		cpu_val;
	unsigned int		cpu_mask;
	const char		*arch_name;
	const char		*elf_name;
	unsigned int		elf_hwcap;
};

#else	/* __KERNEL__ */
#include <asm/elf.h>
#warning "Please include asm/elf.h instead"
#endif	/* __KERNEL__ */
#endif
