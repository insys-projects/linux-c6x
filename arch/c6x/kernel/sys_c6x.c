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

#include <asm/setup.h>
#include <asm/segment.h>
#include <asm/traps.h>
#include <asm/ipc.h>
#include <asm/page.h>
#include <asm/uaccess.h>

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

/*
 * dp allocator
 */
#define DP_ALIGN 32
struct dp_segment {
	struct list_head list;

	char name[24];
	unsigned long addr;
};
static struct dp_segment dp_segment_list[16];
static int dp_segment_initialiased[16];
static int dp_segment_last_allocated[16];

asmlinkage unsigned long sys_dp_alloc(const char *name,
				      int area,
				      unsigned long len)
{
	struct list_head *pos;
	struct dp_segment *tmp;

	/* List initialisation if needed */
	if(dp_segment_initialiased[area] == 0) {
		INIT_LIST_HEAD(&dp_segment_list[area].list);
		dp_segment_initialiased[area] = 1;
	}

	/* Find an already allocated library */
	list_for_each(pos, &dp_segment_list[area].list) {
		tmp = list_entry(pos, struct dp_segment, list);
		if(strncmp(name, tmp->name, 23) == 0)
			goto out;
	}

	/* If no library is found, allocate a new segment */
	tmp = kmalloc(sizeof(struct dp_segment), GFP_KERNEL);
	if(tmp == NULL) {
		printk(KERN_ALERT "Cannot allocate memory for dp_alloc\n");
		return -1;
	}
	strncpy(tmp->name, name, 23);
	tmp->name[23] = 0;
	tmp->addr = dp_segment_last_allocated[area];

	list_add(&(tmp->list), &(dp_segment_list[area].list));

	dp_segment_last_allocated[area] += len;
	dp_segment_last_allocated[area] =
		(dp_segment_last_allocated[area] + DP_ALIGN - 1)
		& ~(DP_ALIGN - 1);

out:
	return tmp->addr;
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
 * System calls used by the kernel.
 */
#include <linux/unistd.h>

#ifdef _TMS320C64_PLUS
#define KERNEL_SYSCALL_GENERATE(syscall_num) \
        asm(" MVK " #syscall_num ",B0\n" \
            " SWE")
#else /* _TMS320C64_PLUS */
#define KERNEL_SYSCALL_GENERATE(syscall_num) \
        asm(" .ref _system_call\n" \
            " MVK " #syscall_num ",B0\n" \
            " MVC CSR,B2\n" \
	    " MV B2,B1\n" \
	    " SHL B1,1,B1\n" \
	    " AND -2,B2,B2\n" \
	    " AND 2,B1,B1\n" \
	    " OR  B2,B1,B1\n" \
	    " MVC B1,CSR\n" \
	    " MVKL _system_call,A0\n" \
	    " MVKH _system_call,A0\n" \
	    " B	A0\n" \
	    " MVC B3,IRP\n" \
	    " NOP 4")
#endif /* _TMS320C64_PLUS */

int clone(unsigned int flags, char * usp)
{
        KERNEL_SYSCALL_GENERATE(120);  /* __NR_clone */
}

int kernel_execve(char * file, char ** argvp, char ** envp)
{
        KERNEL_SYSCALL_GENERATE(11);   /* __NR_execve */
}

int exec_memobj(struct binfmt_memobj * exe,
			      char ** argvp,
			      char ** envp)
{
	KERNEL_SYSCALL_GENERATE(235);  /* __NR_exec_memobj */
}
