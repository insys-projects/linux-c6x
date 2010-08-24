/*
 *  linux/include/asm-c6x/binfmt_mem.h
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
#ifndef __ASM_C6X_BINFMT_MEM_H_
#define __ASM_C6X_BINFMT_MEM_H_

struct memobj_frm_struct {
    unsigned int run_start;
	unsigned int text_start;
	unsigned int text_size;
	unsigned int data_start;
	unsigned int data_size;
	unsigned int bss_start;
	unsigned int bss_size;
	unsigned int brk;
	unsigned int stack_top;
};

int exec_memobj(struct memobj_frm_struct * exe, char ** argvp, char ** envp);
int do_exec_memobj (struct memobj_frm_struct *memobj_frm, char **argv, char **envp, struct pt_regs *regs);

#endif /* __ASM_C6X_BINFMT_MEM_H_ */

