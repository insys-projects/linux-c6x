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
extern void *alloca (size_t size);
extern unsigned int csum_partial(const unsigned char * buff,int len, unsigned int sum);
extern unsigned int ip_fast_csum(unsigned char *iph, unsigned int ihl);

/* consistent area handling */
extern void *consistent_alloc(int gfp, size_t size, dma_addr_t *dma_handle);
extern void *pci_alloc_consistent(struct pci_dev *hwdev, size_t size, dma_addr_t *handle);
extern void consistent_free(void *vaddr, size_t size, dma_addr_t handle);
extern void consistent_sync(void *vaddr, size_t size, int direction);

/******************************************************************************/

/*
 * libcl6x/libgcc functions - used internally by the compiler...
 */
extern int __c6xabi_divi(int dividend, int divisor);
EXPORT_SYMBOL(__c6xabi_divi);

extern unsigned __c6xabi_divu(unsigned  dividend, unsigned divisor);
EXPORT_SYMBOL(__c6xabi_divu);

extern int __c6xabi_remi(int dividend, int divisor);
EXPORT_SYMBOL(__c6xabi_remi);

extern unsigned __c6xabi_remu(unsigned  dividend, unsigned divisor);
EXPORT_SYMBOL(__c6xabi_remu);

extern unsigned long long __c6xabi_mpyll(unsigned long long src1, unsigned long long src2);
EXPORT_SYMBOL(__c6xabi_mpyll);

extern long long __c6xabi_negll(long long src);
EXPORT_SYMBOL(__c6xabi_negll);

extern unsigned long long __c6xabi_llshl(unsigned long long src1, uint src2);
EXPORT_SYMBOL(__c6xabi_llshl);

extern long long __c6xabi_llshr(long long src1, uint src2);
EXPORT_SYMBOL(__c6xabi_llshr);

extern unsigned long long __c6xabi_llshru(unsigned long long src1, uint src2);
EXPORT_SYMBOL(__c6xabi_llshru);

extern void __c6xabi_strasgi(int *dst, const int *src, unsigned cnt);
EXPORT_SYMBOL(__c6xabi_strasgi);

extern void __c6xabi_push_rts(void);
EXPORT_SYMBOL(__c6xabi_push_rts);

extern void __c6xabi_pop_rts(void);
EXPORT_SYMBOL(__c6xabi_pop_rts);

#ifdef CONFIG_TMS320C64XPLUS
extern void __c6xabi_strasgi_64plus(int *dst, const int *src, unsigned cnt);
EXPORT_SYMBOL(__c6xabi_strasgi_64plus);
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
