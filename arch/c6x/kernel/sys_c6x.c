/*
 *  linux/arch/c6x/kernel/sys_c6x.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010, 2011, 2012 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <linux/mutex.h>
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

	if (memory_start <= addr && (addr + size - 1) < max_t(unsigned long, memory_end, dma_memory_end))
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

/* 
 * sys_cacheflush -- flush (part of) the processor cache.
 */
asmlinkage int
sys_cacheflush  (unsigned long addr, unsigned long bytes, unsigned int cache)
{
	/* Sync caches over given range */
	L1D_cache_block_writeback_invalidate(addr, addr + bytes);
	L1P_cache_block_invalidate(addr, addr + bytes);

	return 0;
}

/*
 * TLS syscalls for getting user helper address and set TLS for current thread
 */
unsigned long __user_helper_addr = 0;

asmlinkage unsigned long
sys_get_user_helper (void)
{
	if (!__user_helper_addr) {
		/* Allocate user_helper */
		__user_helper_addr = (unsigned long) kmalloc(PAGE_SIZE, GFP_KERNEL);
	}
	return __user_helper_addr;
}

asmlinkage int
sys_set_tls (unsigned long tls_value)
{
	if (unlikely(!__user_helper_addr))
		return -ENOMEM;
	*((unsigned long *) __user_helper_addr) = tls_value;

	/*
	 * Normally thread.tls is updated during resume() but it needs to be set 
	 * there too for the vfork() special case.
	 */
	current->thread.tls = tls_value;

	return 0;
}

asmlinkage int sys_getpagesize(void)
{
	return PAGE_SIZE;
}

/* 
 * Syscall for DSBT index management
 */
#define DSBT_IDX_MAX        128
#define LIBRARY_NAME_LENGTH 64

struct dsbt_idx {
	struct list_head list;
	char             name[LIBRARY_NAME_LENGTH];
	unsigned long    idx;
};

static DEFINE_MUTEX(__dsbt_idx_mutex);
static unsigned long  __dsbt_idx[DSBT_IDX_MAX >> 5];
static struct list_head *__dsbt_idx_list = NULL;

asmlinkage int sys_dsbt_idx_alloc(const char   *name,
	                          unsigned long start,
				  unsigned long end,
				  unsigned long op)
{
	int res;
	unsigned long idx;
	unsigned long n_idx = 0;
	struct list_head *pos;
	struct dsbt_idx *entry = NULL;

	if ((start >= DSBT_IDX_MAX) || (end >= DSBT_IDX_MAX))
		return -EINVAL;

	mutex_lock(&__dsbt_idx_mutex);

	if (__dsbt_idx_list == NULL) {
		/* Initialize list */
		pos = kmalloc(sizeof(struct list_head), GFP_KERNEL);
		if (pos == NULL) {
			printk(KERN_ALERT "Cannot allocate memory for dsbt_idx_alloc\n");
			res = -ENOMEM;
			goto error;
		}

		__dsbt_idx_list = pos;
		INIT_LIST_HEAD(__dsbt_idx_list);
	} else {
		/* Find an already allocated library */
		list_for_each(pos, __dsbt_idx_list) {
			entry = list_entry(pos, struct dsbt_idx, list);
			if (strncmp(name, entry->name,
				    LIBRARY_NAME_LENGTH - 1) == 0)
				goto found;
		}
	}

	/* If no library is found, allocate a new index entry */
	entry = kmalloc(sizeof(struct dsbt_idx), GFP_KERNEL);
	if (entry == NULL) {
		printk(KERN_ALERT "Cannot allocate memory for dsbt_idx_alloc\n");
		res = -ENOMEM;
		goto error;
	}

	strncpy(entry->name, name,  LIBRARY_NAME_LENGTH - 1);
	entry->name[ LIBRARY_NAME_LENGTH - 1] = 0;

	switch(op) {
	case 0:
		/* Alloc */
		if (end < start) {
			kfree(entry);
			res = -EINVAL;
			goto error;
		}

		/* Allocate from the end to avoid collision with static index */
		idx = end;
		n_idx = start;
		while (1) {
			n_idx = find_next_zero_bit(&__dsbt_idx[0], end, n_idx);
			if (n_idx == end) {
				break;
			}
			idx = n_idx;
			n_idx++;
		}

		if (idx >= end) {
			kfree(entry);
			res = -EBUSY;
			goto error;
		}

		if (test_and_set_bit(idx, &__dsbt_idx[0])) {
			kfree(entry);
			res = -EBUSY;
			goto error;
		}
		break;
	case 1:
		/* Reserve, end is not used */
		idx = start;
		if (test_and_set_bit(idx, &__dsbt_idx[0])) {
			kfree(entry);
			res = -EBUSY;
			goto error;
		}
		break;
	default:
		res = -EINVAL;
		goto error;
	}

	/* Set the new idx */
	entry->idx = idx;
	list_add(&(entry->list), __dsbt_idx_list);

found:
	if (op == 2) {
		/* Free */
		if (!(test_and_clear_bit(start, &__dsbt_idx[0]))) {
			res = -EBUSY;
			goto error;
		}

		if (entry) {
			/* Remove this entry from the list */
			list_del(&(entry->list));
			kfree(entry);
		}
	} 

	res = (int) entry->idx;
error:
	mutex_unlock(&__dsbt_idx_mutex);

	return res;
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
#if defined(CONFIG_TMS320C64XPLUS) || defined(CONFIG_TMS320C66X)
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
