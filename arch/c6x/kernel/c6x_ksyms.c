/*
 *  linux/arch/c6x/kernel/c6x_ksyms.c
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
 */
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/pgtable.h>
#include <asm/unaligned.h>

/* platform dependent support */
struct pt_regs;
struct user;
extern void dump_thread(struct pt_regs * regs, struct user * dump);
extern void *alloca (size_t size);
extern unsigned int csum_partial(const unsigned char * buff,int len, unsigned int sum);
extern unsigned int ip_fast_csum(unsigned char *iph, unsigned int ihl);

/*
 * libcl6x functions - functions that are used internally by the
 * compiler...
 */
/* ANSI concatenation macros.  */
#define CONCAT1(a, b) CONCAT2(a, b)
#define CONCAT2(a, b) a ## b

/* use the right prefix */
#ifdef __TI_EABI__
#define SYM(x) CONCAT1 (__c6xabi,x)
#else
#define SYM(x) x
#endif

extern int SYM(_divi) (int dividend, int divisor);
extern unsigned SYM(_divu) (unsigned  dividend, unsigned divisor);
extern int SYM(_remi) (int dividend, int divisor);
extern unsigned SYM(_remu) (unsigned  dividend, unsigned divisor);
extern unsigned long long SYM(_mpyll)(unsigned long long src1, unsigned long long src2);
extern long long SYM(_negll)(long long src);
extern unsigned long long SYM(_llshl)(unsigned long long src1, uint src2);
extern long long SYM(_llshr)(long long src1, uint src2);
extern unsigned long long SYM(_llshru)(unsigned long long src1, uint src2);
extern void SYM(_strasgi)(int *dst, const int *src, unsigned cnt);
extern void SYM(_push_rts)(void);
extern void SYM(_pop_rts)(void);
#ifdef CONFIG_TMS320C64XPLUS
extern void SYM(_strasgi_64plus)(int *dst, const int *src, unsigned cnt);
#endif

#ifndef __TI_EABI__
extern void SYM(_strasg)(int *dst, const int *src, unsigned cnt);
#endif

/* consistent area handling */
extern void *consistent_alloc(int gfp, size_t size, dma_addr_t *dma_handle);
extern void *pci_alloc_consistent(struct pci_dev *hwdev, size_t size, dma_addr_t *handle);
extern void consistent_free(void *vaddr, size_t size, dma_addr_t handle);
extern void consistent_sync(void *vaddr, size_t size, int direction);

/******************************************************************************/

/* address translation */
EXPORT_SYMBOL(mm_ptov);
EXPORT_SYMBOL(mm_vtop);

/*
 * libcl6x functions - functions that are used internally by the
 * compiler...
 */
EXPORT_SYMBOL(SYM(_divi));
EXPORT_SYMBOL(SYM(_divu));
EXPORT_SYMBOL(SYM(_remi));
EXPORT_SYMBOL(SYM(_remu));
EXPORT_SYMBOL(SYM(_mpyll));
EXPORT_SYMBOL(SYM(_negll));
EXPORT_SYMBOL(SYM(_llshl));
EXPORT_SYMBOL(SYM(_llshr));
EXPORT_SYMBOL(SYM(_llshru));
EXPORT_SYMBOL(SYM(_strasgi));
EXPORT_SYMBOL(SYM(_push_rts));
EXPORT_SYMBOL(SYM(_pop_rts));
#ifdef CONFIG_TMS320C64XPLUS
EXPORT_SYMBOL(SYM(_strasgi_64plus));
#endif

#ifndef __TI_EABI__
EXPORT_SYMBOL(SYM(_strasg));
#endif

/* i/o functions */
EXPORT_SYMBOL(__raw_writesb);
EXPORT_SYMBOL(__raw_writesw);
EXPORT_SYMBOL(__raw_writesl);
EXPORT_SYMBOL(__raw_readsb);
EXPORT_SYMBOL(__raw_readsw);
EXPORT_SYMBOL(__raw_readsl);

/* string / mem functions */
EXPORT_SYMBOL(memset);
EXPORT_SYMBOL(memcpy);
EXPORT_SYMBOL(memmove);
EXPORT_SYMBOL(memcmp);
EXPORT_SYMBOL(_c6x_delay);

/* platform dependent support */
EXPORT_SYMBOL(current_text_addr);
EXPORT_SYMBOL(irq_IER);
EXPORT_SYMBOL(get_current);
EXPORT_SYMBOL(alloca);
EXPORT_SYMBOL(__current_thread_info);
EXPORT_SYMBOL(csum_partial);
EXPORT_SYMBOL(ip_fast_csum);

/* consistent area handling */
EXPORT_SYMBOL(pci_alloc_consistent);
EXPORT_SYMBOL(consistent_alloc);
EXPORT_SYMBOL(consistent_free);
EXPORT_SYMBOL(consistent_sync);

/* minimal access checks for no-mmu */
extern int _access_ok(unsigned long addr, unsigned long size);
EXPORT_SYMBOL(_access_ok);

/*
 * unaligned access
 *
 * These should really be macros or inline asm but the
 * TI toolchain doesn't have good enough asm() support.
 */ 
EXPORT_SYMBOL(get_unaligned_le64);
EXPORT_SYMBOL(get_unaligned_be64);
EXPORT_SYMBOL(get_unaligned_le32);
EXPORT_SYMBOL(get_unaligned_be32);
EXPORT_SYMBOL(put_unaligned_le64);
EXPORT_SYMBOL(put_unaligned_be64);
EXPORT_SYMBOL(put_unaligned_le32);
EXPORT_SYMBOL(put_unaligned_be32);
