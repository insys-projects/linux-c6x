/*
 *  linux/include/asm-c6x/pgtable.h
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
#ifndef __ASM_C6X_PGTABLE_H
#define __ASM_C6X_PGTABLE_H

#include <asm-generic/4level-fixup.h>

#include <asm/setup.h>
#include <asm/page.h>

#define pgd_present(pgd)      (1)       /* pages are always present on NO_MM */
#define kern_addr_valid(addr) (1)

#define PAGE_NONE		__pgprot(0)    /* these mean nothing to NO_MM */
#define PAGE_SHARED		__pgprot(0)    /* these mean nothing to NO_MM */
#define PAGE_COPY		__pgprot(0)    /* these mean nothing to NO_MM */
#define PAGE_READONLY	        __pgprot(0)    /* these mean nothing to NO_MM */
#define PAGE_KERNEL		__pgprot(0)    /* these mean nothing to NO_MM */
#define pgprot_noncached(prot)	(prot)

extern void paging_init(void);
#define swapper_pg_dir ((pgd_t *) 0)

#define __swp_type(x)		(((x).val >> 2) & 0x7f)
#define __swp_offset(x)		((x).val >> 9)
#define __swp_entry(type,offset) ((swp_entry_t) { ((type) << 2) | ((offset) << 9) })
#define __pte_to_swp_entry(pte)	((swp_entry_t) { pte_val(pte) })
#define __swp_entry_to_pte(swp)	((pte_t) (swp).val)

static inline int pte_file(pte_t pte)
{
	return 0;
}

/*
 * ZERO_PAGE is a global shared page that is always zero: used
 * for zero-mapped memory areas etc..
 */
#define ZERO_PAGE(vaddr)	(virt_to_page(0))

/*
 * No page table caches to initialise
 */
#define pgtable_cache_init()   do { } while (0)
#define io_remap_page_range(vma, vaddr, paddr, size, prot)		\
		remap_pfn_range(vma, vaddr, (paddr) >> PAGE_SHIFT, size, prot)

#define io_remap_pfn_range(vma, vaddr, pfn, size, prot)		\
		remap_pfn_range(vma, vaddr, pfn, size, prot)

extern unsigned long get_fb_unmapped_area(struct file *, unsigned long,
					  unsigned long, unsigned long,
					  unsigned long);
#define HAVE_ARCH_FB_UNMAPPED_AREA

#define get_videoin_unmapped_area get_fb_unmapped_area

#define arch_start_context_switch(prev)	do {} while (0)

#endif /* __ASM_C6X_PGTABLE_H */
