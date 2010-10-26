/*
 *  linux/arch/c6x/kernel/sys_c6x.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This file contains various random system calls that
 *  have a non-standard calling sequence on the C6x platform.
 */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/sem.h>
#include <linux/msg.h>
#include <linux/shm.h>
#include <linux/stat.h>
#include <linux/slab.h>
#include <linux/mman.h>
#include <linux/file.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/mtd/map.h>
#include <linux/stringify.h>

#include <asm/setup.h>
#include <asm/segment.h>
#include <asm/traps.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>

#ifdef CONFIG_ACCESS_CHECK
int _access_ok(unsigned long addr, unsigned long size)
{
	if (!size)
		return 1;

	if (!addr || addr > (0xffffffffUL - (size - 1)))
		goto _bad_access;

	if (segment_eq(get_fs(), KERNEL_DS))
		return 1;

	if (memory_start <= addr && (addr + size) < memory_end)
		return 1;

_bad_access:
#if 0
	printk(KERN_CRIT "**Bad access attempt: pid[%d] addr[%p] size[0x%x]\n", current->pid, addr, size);
#endif
	return 0;
}
#endif

unsigned long get_fb_unmapped_area(struct file *filp,
				   unsigned long orig_addr,
				   unsigned long len,
				   unsigned long pgoff,
				   unsigned long flags)
{
	return 0;
}

EXPORT_SYMBOL(get_fb_unmapped_area);

asmlinkage long sys_ioperm(unsigned long from, unsigned long num, int on)
{
	return -ENOSYS;
}

/* sys_cacheflush -- flush (part of) the processor cache.  */
asmlinkage int
sys_cacheflush (unsigned long addr, int scope, int cache, unsigned long len)
{
	return 0;
}

/* sys_cache_sync -- sync caches over given range */
asmlinkage int
sys_cache_sync (unsigned long s, unsigned long e)
{
	/* FIXME. Add range checks */

	L1D_cache_block_writeback_invalidate(s, e);
	L1P_cache_block_invalidate(s, e);

	return 0;
}

asmlinkage int sys_getpagesize(void)
{
	return PAGE_SIZE;
}

asmlinkage long old_mmap(unsigned long addr, unsigned long len,
			 unsigned long prot, unsigned long flags,
			 unsigned long fd, unsigned long offset)
{
	if (offset & ~PAGE_MASK)
		return -EINVAL;
	return sys_mmap_pgoff(addr, len, prot, flags, fd, offset >> PAGE_SHIFT);
}


struct sel_arg_struct {
	unsigned long n;
	fd_set *inp;
	fd_set *outp;
	fd_set *exp;
	struct timeval *tvp;
};

asmlinkage int old_select(struct sel_arg_struct __user *arg)
{
	struct sel_arg_struct a;

	if (copy_from_user(&a, arg, sizeof(a)))
		return -EFAULT;
	/* sys_select() does the appropriate kernel locking */
	return sys_select(a.n, a.inp, a.outp, a.exp, a.tvp);
}

/*
 * This is inlined from a C file to avoid incompatibilities between
 * the GNU and TI assembler syntax.
 */
   asm ("	.global kernel_execve\n"
	"kernel_execve:\n"
	" 	MVK	.S2	" __stringify(__NR_execve) ",B0\n"
#ifdef CONFIG_TMS320C64XPLUS
	"	SWE\n"
	"	BNOP	.S2	B3,5\n"
#else
        " 	MVC	.S2	CSR,B2\n"
	"	SHL	.S2	B2,1,B1\n"
	" ||	AND	.D2	-2,B2,B2\n"
	"	AND	.D2	2,B1,B1\n"
	" ||	MVKL	.S1	_system_call,A0\n"
	"	OR	.D2	B2,B1,B1\n"
	" ||	MVKH	.S1	_system_call,A0\n"
	"	MVC	.S2	B1,CSR\n"
	"	B	.S2X	A0\n"
	"	MVC	.S2	B3,IRP\n"
	"	NOP	4"
#endif
	   );
