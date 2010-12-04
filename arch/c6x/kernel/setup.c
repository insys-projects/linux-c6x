/*
 *  linux/arch/c6x/kernel/setup.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2006, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This file handles the architecture-dependent parts of system setup
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/genhd.h>
#include <linux/errno.h>
#include <linux/console.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/seq_file.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mm.h>

#ifdef CONFIG_MTD_UCLINUX
#include <linux/mtd/map.h>
#include <linux/ext2_fs.h>
#include <linux/cramfs_fs.h>
#include <linux/romfs_fs.h>
#endif

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/io.h>
#include <asm/cache.h>
#include <asm/pm.h>
#include <asm/hardware.h>

#include <mach/board.h>

#ifdef CONFIG_BLK_DEV_INITRD
#include <linux/initrd.h>
#include <asm/pgtable.h>
#endif

#include "tags.h"

extern unsigned int  _stext, _etext, _edata, _bss_start, _bss_end;
extern unsigned long zone_dma_start, zone_dma_size;

unsigned int memory_start, memory_end; 
unsigned int c6x_early_uart_cons = 0;

static char c6x_command_line[COMMAND_LINE_SIZE];
static char default_command_line[COMMAND_LINE_SIZE] __section(.cmdline) = CONFIG_CMDLINE;
static const char *cpu_name, *cpu_voltage, *mmu, *fpu, *soc_rev;
static char __cpu_rev[5], *cpu_rev;
static size_t initrd_size = CONFIG_BLK_DEV_RAM_SIZE*1024;
#ifdef CONFIG_TMS320C64XPLUS
static unsigned int cpu_num = 0;
#endif

#if defined(CONFIG_MTD_PLATRAM) || defined(CONFIG_MTD_PLATRAM_MODULE)
unsigned int c6x_platram_start;
unsigned int c6x_platram_size = 0;
#endif

#if defined(CONFIG_VGA_CONSOLE)
struct screen_info screen_info;
#endif

/*
 * Mach dep functions
 */ 
void (*mach_reset)(void) = NULL;
int  (*mach_set_clock_mmss)(unsigned long) = NULL;
void (*mach_debug_init)(void) = NULL;
void (*mach_gettod)(int*, int*, int *, int*, int*, int *) = NULL;
int  (*mach_request_irq) (unsigned int, void (*handler)(int, void *, struct pt_regs *),
			  unsigned long, const char *, void *) = NULL;
int  (*mach_free_irq) (unsigned int, void *) = NULL;
void (*mach_leds_timer)(void) = NULL;
void (*mach_setup_timer)(void) = NULL;
void (*mach_init_IRQ) (void) = NULL;
void (*mach_progress) (unsigned int, char *) = NULL;
void (*mach_print_value) (char *, unsigned long) = NULL;

struct tag_header *c6x_tags_pointer __initdata;

static unsigned long dummy_gettimeoffset(void)
{
	return 0;
}

unsigned long (*mach_gettimeoffset)(void) = dummy_gettimeoffset;;

void get_cpuinfo(void)
{
	unsigned cpu_id, rev_id, csr;

	csr = get_creg(CSR);
	cpu_id = csr >> 24;
	rev_id = (csr >> 16) & 0xff;

	mmu         = "none";
	cpu_voltage = "unknown";

	switch (cpu_id) {
	case 0:
		cpu_name = "C67x";
		fpu = "yes";
		break;
	case 2:
		cpu_name = "C62x";
		fpu = "none";
		break;
	case 8:
		cpu_name = "C64x";
		fpu = "none";
		break;
	case 12:
		cpu_name = "C64x";
		fpu = "none";
		break;
	case 16:
		cpu_name = "C64x+";
		cpu_voltage = "1.2V";
		fpu = "none";
		break;
	default:
		cpu_name = "unknown";
		fpu = "none";
	}
		
	if (cpu_id < 16) {
		switch (rev_id) {
		case 0x1:
			if (cpu_id > 8) {
				cpu_rev = "DM640/DM641/DM642/DM643";
				cpu_voltage = "1.2V - 1.4V";
			} else {
				cpu_rev = "C6201";
				cpu_voltage = "2.5V";
			}
			break;
		case 0x2:
			cpu_rev = "C6201B/C6202/C6211";
			cpu_voltage = "1.8V";
			break;
		case 0x3:
			cpu_rev = "C6202B/C6203/C6204/C6205";
			cpu_voltage = "1.5V";
			break;
		case 0x201:
			cpu_rev = "C6701 revision 0 (early CPU)";
			cpu_voltage = "1.8V";
			break;
		case 0x202:
			cpu_rev = "C6701/C6711/C6712";
			cpu_voltage = "1.8V";
			break;
		case 0x801:
			cpu_rev = "C64x";
			cpu_voltage = "1.5V"; 
			break;
		default:
			cpu_rev = "unknown";
		}
	} else {
		cpu_rev = __cpu_rev;
		snprintf(__cpu_rev, sizeof(__cpu_rev), "0x%x", cpu_id);
	}

#ifndef CONFIG_TMS320C64XPLUS
	printk("CPU: %s revision %s core voltage %s\n",
	       cpu_name, cpu_rev, cpu_voltage);
#else
	cpu_num = get_creg(DNUM) & 0xff;
	printk("CPU: %s revision %s core voltage %s core number %d\n",
	       cpu_name, cpu_rev, cpu_voltage, cpu_num);
#endif

#ifdef C6X_SOC_HAS_CORE_REV
	soc_rev = arch_compute_silicon_rev(arch_get_silicon_rev());
#else
	soc_rev = "unknown";
#endif
}

#ifdef CONFIG_TMS320C6X_CACHES_ON
/*
 * L1 and L2 caches configuration
 */
static void cache_init(void)
{
	/* Set L2 caches on the the whole L2 SRAM memory */
	L2_cache_set_mode(L2MODE_SIZE);

	/* Enable L1 */
	L1_cache_on();
}
#endif /* CONFIG_TMS320C6X_CACHES_ON */

static void cache_set(unsigned int start, unsigned int end)
{
	/* Set whole external memory cacheable */
	enable_caching((unsigned int *) start, (unsigned int *) (end - 1));
}

/*
 * Early parsing of the command line
 */
static void __init parse_cmdline_early (char ** cmdline_p)
{
	char c = ' ', *to = c6x_command_line, *from = boot_command_line;
	int len = 0;
	int userdef = 0;

	for (;;) {
		if (c != ' ')
			goto next_char;

		if (!memcmp(from, "mem=", 4)) {
			unsigned long mem_size;

			if (to != c6x_command_line)
				to--;
			
			/* If the user specifies memory size, we
			 * limit the default memory map to
			 * that size. mem=number can be used to
			 * trim the existing memory map.
			 */
			unsigned long mem_size;
			
			mem_size = (unsigned long) memparse(from + 4, &from);
			memory_end = PAGE_ALIGN(REGION_START(&_stext) + mem_size);

			userdef = 1;
		}

		else if (!memcmp(from, "memdma=", 7)) {
			if (to != c6x_command_line)
				to--;
			
			zone_dma_size = (unsigned long) memparse(from + 7, &from);
			if (*from == '@') {
				zone_dma_start = memparse(from + 1, &from);
				userdef = 1;
			}
		} else if (!memcmp(from, "console=ttyS", 12)) {
			/* This is for the case we want to use a very early UART console */
			c6x_early_uart_cons = 1;
		}

#ifdef CONFIG_BLK_DEV_INITRD
		else if (!memcmp(from, "initrd=", 7)) {
			if (to != c6x_command_line)
				to--;
            
			initrd_start = memparse(from + 7, &from);
			if (*from == ',') {
				initrd_size = memparse(from + 1, &from);
			}
		}
#endif /* CONFIG_BLK_DEV_INITRD */
		
#if defined(CONFIG_MTD_PLATRAM) || defined(CONFIG_MTD_PLATRAM_MODULE)
		else if (!memcmp(from, "platram=", 8)) {
			if (to != c6x_command_line)
				to--;

			c6x_platram_start = memparse(from + 8, &from);
			if (*from == ',')
				c6x_platram_size = memparse(from + 1, &from);
		}
#endif

	next_char:
		c = *(from++);
		if (!c)
			break;
		if (COMMAND_LINE_SIZE <= ++len)
			break;
		*(to++) = c;
	}
	*to = '\0';
	*cmdline_p = c6x_command_line;
	if (userdef) {
		printk(KERN_INFO "physical RAM map changed by user\n");
	}
}

#ifdef CONFIG_MTD_UCLINUX
extern struct map_info uclinux_ram_map;

static unsigned long get_romfs_size(void *addr)
{
#ifdef CONFIG_ROMFS_FS
	if ((*((unsigned long *)addr) == ROMSB_WORD0) &&
	    (*((unsigned long *)addr + 1) == ROMSB_WORD1))
		return PAGE_ALIGN(be32_to_cpu(*((unsigned long *)addr + 2)));
#endif

#ifdef CONFIG_CRAMFS
	if (*((unsigned long *)addr) == CRAMFS_MAGIC)
		return PAGE_ALIGN(*((unsigned long *)addr + 1));
#endif

#if defined(CONFIG_EXT2_FS) || defined(CONFIG_EXT3_FS)
	if (le16_to_cpu(*((unsigned short *)addr + 0x200 + 28)) == EXT2_SUPER_MAGIC)
		return PAGE_ALIGN(le32_to_cpu((*((unsigned long *)addr + 0x100 + 1))) * 1024);
#endif

	return 0;
}

/*
 * This is called just before .bss is cleared so that any MTD_UCLINUX
 * filesystem located in bss can be moved out of the way first.
 */
void __init c6x_mtd_early_init(void)
{
	unsigned int romfs_size;

	romfs_size = PAGE_ALIGN(get_romfs_size((unsigned *)&_bss_start));

	/* Move ROMFS out of BSS */
	if (romfs_size)
		memmove(&_bss_end, (int *)&_bss_start, romfs_size);
}

#endif /* CONFIG_MTD_UCLINUX */

void __init setup_arch(char **cmdline_p)
{
	unsigned int memory_size;
	int bootmap_size;
	struct tag_cmdline *tcmd;
	char tmp_command_line[COMMAND_LINE_SIZE];
#if defined(CONFIG_MTD_UCLINUX)
	unsigned long romfs_size;
#endif

	if (!c6x_tags_are_valid(c6x_tags_pointer))
		c6x_tags_pointer = NULL;

	/* interrupts must be masked */
	local_irq_disable();

	/* Set the Interrupt Service Table (IST) at the beginning of the 
	   external memory */
	set_ist(REGION_START(&_stext));

#ifdef CONFIG_TMS320C6X_CACHES_ON
	/* Perform caches initialization */
	cache_init();
#endif	

	/* Initialise C6x IRQs */
	init_irq_mask();

	/* Set peripheral power-down */
#ifdef CONFIG_PM
	pwr_pdctl_set(PWR_PDCTL_ALL);
#endif

  	/* Call SOC configuration function */
	c6x_soc_setup_arch();

  	/* Call board configuration function */
	c6x_board_setup_arch();

	/* Get CPU info */
	get_cpuinfo();

	/* Memory management */
	printk("Initializing kernel\n");
	mach_progress(2, "Initialize memory");

#if defined(CONFIG_RAM_ATTACHED_ROMFS)
	memory_start = PAGE_ALIGN((unsigned int) RAM_START);
#elif defined(CONFIG_MTD_UCLINUX)
	romfs_size = get_romfs_size(&_bss_end);
	memory_start = PAGE_ALIGN(((unsigned int) &_bss_end) + romfs_size);

	uclinux_ram_map.phys = (unsigned long)&_bss_end;
	uclinux_ram_map.size = romfs_size;
#else  
	memory_start = PAGE_ALIGN((unsigned int) &_bss_end);
#endif  

	memory_end   = PAGE_ALIGN((unsigned int) REGION_START(&_stext) + BOARD_RAM_SIZE);
	memory_size  = (memory_end - memory_start);

	mach_print_value("memory_start:", memory_start);
	mach_print_value("memory_end  :", memory_end);
	mach_print_value("memory_size :", memory_size);

	init_mm.start_code = (unsigned long) &_stext;
	init_mm.end_code   = (unsigned long) &_etext;
#if defined(CONFIG_RAM_ATTACHED_ROMFS)
	init_mm.end_data   = (unsigned long) RAM_START;
	init_mm.brk        = (unsigned long) RAM_START;
#elif defined(CONFIG_MTD_UCLINUX)
	init_mm.end_data   = (unsigned long) (((unsigned long) &_bss_end) + romfs_size);
	init_mm.brk        = (unsigned long) (((unsigned long) &_bss_end) + romfs_size);
#else
	init_mm.end_data   = (unsigned long) &_edata;
	init_mm.brk        = (unsigned long) &_bss_end;
#endif

	/* Initialize command line */
	mach_progress(4, "Initialize command line");

	strlcpy(tmp_command_line, default_command_line, COMMAND_LINE_SIZE);

	*cmdline_p = tmp_command_line;

	/* Let cmdline passed through tag array override CONFIG_CMDLINE */
	tcmd = c6x_tag_find(c6x_tags_pointer, TAG_CMDLINE);
	if (tcmd)
		strlcpy(tmp_command_line, tcmd->cmdline, COMMAND_LINE_SIZE);

	strcpy(boot_command_line, *cmdline_p);
	parse_cmdline_early(cmdline_p);

	/* Set caching of external RAM used by Linux */
	cache_set((unsigned long) REGION_START(&_stext), memory_end);

	/*
	 * give all the memory to the bootmap allocator,  tell it to put the
	 * boot mem_map at the start of memory
	 */
	mach_progress(5, "Initialize bootmap allocator");
	bootmap_size = init_bootmem_node(NODE_DATA(0),
					 memory_start >> PAGE_SHIFT,
					 PAGE_OFFSET >> PAGE_SHIFT,
					 memory_end >> PAGE_SHIFT);

	/*
	 * free the usable memory,  we have to make sure we do not free
	 * the bootmem bitmap so we then reserve it after freeing it :-)
	 */
	mach_progress(6, "Free usable memory");
	free_bootmem(memory_start, memory_end - memory_start);
	reserve_bootmem(memory_start, bootmap_size, BOOTMEM_DEFAULT);

#if defined(CONFIG_MTD_PLATRAM) || defined(CONFIG_MTD_PLATRAM_MODULE)
	if (c6x_platram_size) {
		if (c6x_platram_start < (memory_start + bootmap_size) ||
		    (c6x_platram_start + c6x_platram_size) > memory_end) {
			printk(KERN_ERR "Invalid platram= argument. Out of range %p - %p!\n",
			       (void *)memory_start, (void *)memory_end);
			c6x_platram_size = 0;
		} else
			reserve_bootmem(c6x_platram_start, c6x_platram_size, BOOTMEM_DEFAULT);
	}
#endif

#ifdef CONFIG_BLK_DEV_INITRD
	if (initrd_start) {
		if (initrd_start >= memory_start && initrd_start + initrd_size <= memory_end) {
			reserve_bootmem(initrd_start, initrd_size, BOOTMEM_DEFAULT);
			initrd_end = initrd_start+initrd_size;
		}
		else {
			printk(KERN_ERR "initrd is not contained in normal memory\n"
			       "initrd=(0x%08lx:0x%08lx) normal_mem=(%p:%p)\n"
			       "disabling initrd\n",
			       initrd_start, initrd_start + initrd_size,
			       (void *)memory_start, (void *)memory_end);
			initrd_start = 0;
		}
	}
	else {
		printk(KERN_INFO "no initrd specified\n");
	}
#endif

	/*
	 * get kmalloc into gear
	 */
	mach_progress(7, "Initializing paging");
	paging_init();

	mach_progress(8, "End of C6x arch dep initialization");

#if defined(CONFIG_VT) && defined(CONFIG_DUMMY_CONSOLE)
	conswitchp = &dummy_con;
#endif
}

static int show_cpuinfo(struct seq_file *m, void *v)
{
	unsigned long clock_freq = ((loops_per_jiffy<<1)+(500000/HZ))
		/((500000<<1)/HZ);
#ifndef CONFIG_TMS320C64XPLUS
	seq_printf(m, 
		   "CPU:\t\t%s\n"
		   "Core revision:\t%s\n"
		   "Core voltage:\t%s\n"
		   "MMU:\t\t%s\n"
		   "FPU:\t\t%s\n"
		   "Clocking:\t%luMHz\n"
		   "BogoMips:\t%lu.%02lu\n"
		   "Calibration:\t%lu loops\n",
		   cpu_name, cpu_rev, cpu_voltage, mmu, fpu,
		   clock_freq,
		   (loops_per_jiffy/(500000/HZ)),(loops_per_jiffy/(5000/HZ))%100,
		   loops_per_jiffy);
#else
	seq_printf(m, 
		   "CPU:\t\t%s\n"
		   "Core revision:\t%s\n"
		   "Core voltage:\t%s\n"
		   "Core num:\t%d\n"
		   "MMU:\t\t%s\n"
		   "FPU:\t\t%s\n"
		   "Silicon rev:\t%s\n"
		   "Clocking:\t%luMHz\n"
		   "BogoMips:\t%lu.%02lu\n"
		   "Calibration:\t%lu loops\n",
		   cpu_name, cpu_rev, cpu_voltage, cpu_num, mmu, fpu,
		   soc_rev, clock_freq,
		   (loops_per_jiffy/(500000/HZ)),(loops_per_jiffy/(5000/HZ))%100,
		   loops_per_jiffy);
#endif
	return 0;
}

static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}
static void c_stop(struct seq_file *m, void *v)
{
}

struct seq_operations cpuinfo_op = {
	c_start,
	c_stop,
	c_next,
	show_cpuinfo
};

void arch_gettod(int *year, int *mon, int *day, int *hour, int *min, int *sec)
{
	if (mach_gettod)
		mach_gettod(year, mon, day, hour, min, sec);
	else
		*year = *mon = *day = *hour = *min = *sec = 0;
}
