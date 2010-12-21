/*
 *  linux/arch/c6x/mm/init.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/bootmem.h>
#ifdef CONFIG_BLK_DEV_RAM
#include <linux/blkdev.h>
#endif

#include <asm/setup.h>
#include <asm/segment.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/machdep.h>
#include <asm/uaccess.h>
#include <asm/cache.h>

extern void die_if_kernel(char *,struct pt_regs *,long);
extern void show_net_buffers(void);
extern void free_initmem(void);

#ifdef CONFIG_PGCACHE_LIMIT
extern unsigned long pgcache_limit;
#endif
/*
 * BAD_PAGE is the page that is used for page faults when linux
 * is out-of-memory. Older versions of linux just did a
 * do_exit(), but using this instead means there is less risk
 * for a process dying in kernel mode, possibly leaving a inode
 * unused etc..
 *
 * BAD_PAGETABLE is the accompanying page-table: it is initialized
 * to point to BAD_PAGE entries.
 *
 * ZERO_PAGE is a special page that is used for zero-initialized
 * data and COW.
 */
static unsigned long empty_bad_page_table;
static unsigned long empty_bad_page;
unsigned long empty_zero_page;
unsigned long rom_length;

/*
 * DMA zone management, can be redefined using the memdma= kernel command line
 */
unsigned long zone_dma_start = 0; /* at the end of the physical memory */
unsigned long zone_dma_size  = 0; /* none by default */

void show_mem(void)
{
    unsigned long i;
    int free = 0, total = 0, reserved = 0, shared = 0;

    printk("\nMem-info:\n");
    show_free_areas();
    i = max_mapnr;
    while (i-- > 0) {
	total++;
	if (PageReserved(mem_map+i))
	    reserved++;
	else if (!page_count(mem_map+i))
	    free++;
	else
	    shared += page_count(mem_map+i) - 1;
    }
    printk("%d pages of RAM\n",total);
    printk("%d free pages\n",free);
    printk("%d reserved pages\n",reserved);
    printk("%d pages shared\n",shared);
}

/*
 * paging_init() continues the virtual memory environment setup which
 * was begun by the code in arch/head.S.
 * The parameters are pointers to where to stick the starting and ending
 * addresses  of available kernel virtual memory.
 */
void paging_init(void)
{
	struct pglist_data *pgdat = NODE_DATA(0);
	unsigned long zones_size[MAX_NR_ZONES] = {0, };

	/*
	 * initialize the bad page table and bad page to point
	 * to a couple of allocated pages
	 */
	empty_bad_page_table = (unsigned long) alloc_bootmem_pages(PAGE_SIZE);
	empty_bad_page       = (unsigned long) alloc_bootmem_pages(PAGE_SIZE);
	empty_zero_page      = (unsigned long) alloc_bootmem_pages(PAGE_SIZE);
	memset((void *)empty_zero_page, 0, PAGE_SIZE);

	/*
	 * Set up user data space
	 */
	set_fs(KERNEL_DS);

	/*
	 * Define the DMA and non-DMA zones
	 */
	if (zone_dma_size != 0) {
		zone_dma_start = zone_dma_start ? zone_dma_start :
			(memory_end - zone_dma_size) & ~(IMCR_MAR_SIZE - 1);

		zone_dma_size  = (memory_end - zone_dma_start);
		printk(KERN_INFO "Zone DMA start=0x%x size=0x%x\n", 
		       (unsigned int) zone_dma_start,
		       (unsigned int) zone_dma_size);
		
		disable_caching((unsigned int *) zone_dma_start,
				(unsigned int *) (zone_dma_start + 
						  (((zone_dma_size) + IMCR_MAR_SIZE - 1) 
						   & ~(IMCR_MAR_SIZE - 1))) - 1);
		
		printk(KERN_INFO "disabling caching for 0x%x to 0x%x\n",
		       (unsigned int) zone_dma_start,
		       (unsigned int) (zone_dma_start + (((zone_dma_size) + IMCR_MAR_SIZE - 1) 
							 & ~(IMCR_MAR_SIZE - 1)) - 1));
		       }

	zones_size[ZONE_DMA]     = zone_dma_size >> PAGE_SHIFT;
	zones_size[ZONE_NORMAL]  = 
		(memory_end - PAGE_OFFSET - zone_dma_size) >> PAGE_SHIFT;

	pgdat->node_zones[ZONE_DMA].zone_start_pfn =
		zone_dma_start >> PAGE_SHIFT;
	pgdat->node_zones[ZONE_NORMAL].zone_start_pfn = 
		__pa(PAGE_OFFSET) >> PAGE_SHIFT;

	free_area_init(zones_size);
}

void mem_init(void)
{
	int codek = 0, datak = 0, initk = 0;
	extern unsigned int _etext, _stext, _sdata, _ebss, __init_begin, __init_end;

	unsigned long tmp;
	unsigned long start_mem = memory_start;
	unsigned long end_mem   = memory_end;
	unsigned long len       = end_mem - start_mem;

#ifdef DEBUG
	printk("mem_init: start=%lx, end=%lx\n", start_mem, end_mem);
#endif

	end_mem &= PAGE_MASK;
	high_memory = (void *) end_mem;

	start_mem = PAGE_ALIGN(start_mem);
	max_mapnr = num_physpages = MAP_NR(high_memory);

	/* this will put all memory onto the freelists */
	totalram_pages = free_all_bootmem();

	codek = (&_etext - &_stext) >> 10;
	datak = (&_ebss - &_sdata) >> 10;
	initk = (&__init_begin - &__init_end) >> 10;

	tmp = nr_free_pages() << PAGE_SHIFT;
	printk("Memory available: %luk/%luk RAM, %luk/%luk ROM (%dk kernel code, %dk data)\n",
	       tmp >> 10,
	       len >> 10,
	       (rom_length > 0) ? ((rom_length >> 10) - codek) : 0,
	       rom_length >> 10,
	       codek,
	       datak
	       );
#ifdef CONFIG_PGCACHE_LIMIT
	pgcache_limit = nr_free_pages() >> 2; /* Number of pages / 4 */
#endif
}

#ifdef CONFIG_BLK_DEV_INITRD
void free_initrd_mem(unsigned long start, unsigned long end)
{
	int pages = 0;
	for (; start < end; start += PAGE_SIZE) {
		ClearPageReserved(virt_to_page(start));
		init_page_count(virt_to_page(start));
		free_page(start);
		totalram_pages++;
		pages++;
	}
	printk("Freeing initrd memory: %dk freed\n", (pages * PAGE_SIZE) >> 10);
}
#endif

void
free_initmem()
{
	unsigned long addr;
	extern unsigned int __init_begin, __init_end;

	/*
	 * The following code should be cool even if these sections
	 * are not page aligned.
	 */
	addr = PAGE_ALIGN((unsigned long)(&__init_begin));

	/* next to check that the page we free is not a partial page */
	for (; addr + PAGE_SIZE < (unsigned long)(&__init_end); addr +=PAGE_SIZE) {
		ClearPageReserved(virt_to_page(addr));
		init_page_count(virt_to_page(addr));
		free_page(addr);
		totalram_pages++;
	}
	printk("Freeing unused kernel memory: %dK freed\n",
	       (int) ((addr - PAGE_ALIGN((long) &__init_begin)) >> 10));
}
