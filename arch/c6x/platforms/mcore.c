/*
 *  linux/arch/c6x/kernel/mcore.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2007, 2009, 2010, 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 *  Part of the code copied from fs/proc/nommu.c:
 *
 *  Copyright (C) 2004 Red Hat, Inc. All Rights Reserved.
 *  Written by David Howells (dhowells@redhat.com)
 *  
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/backing-dev.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/percpu.h>
#include <asm/dscr.h>
#ifdef ARCH_HAS_MSM
#include <asm/msmc.h>
#endif
#include <mach/board.h>

#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "MCORE: [%s] " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...) 
#endif

static struct file_operations ram_proc_fops;
static struct file_operations control_proc_fops;
static struct backing_dev_info ram_backing_dev_info;

struct rb_root mcore_region_tree = RB_ROOT;
static DECLARE_RWSEM(maps_sem);

struct ram_private_data {
	u32 core_id;
	u32 start;
	u32 size;
};

extern unsigned int memory_end;

/*
 * display a single region to a sequenced file
 */
static int mcore_region_show(struct seq_file *m, struct vm_region *region)
{
	unsigned long ino = 0;
	char *name;
	dev_t dev = 0;
	int flags, len;

	flags = region->vm_flags;
	name  = (char *) region->vm_file;

	dev = 0;
	ino = 0;

	seq_printf(m,
		   "%08lx-%08lx %c%c%c%c %08llx %02x:%02x %lu %n",
		   region->vm_start,
		   region->vm_end,
		   flags & VM_READ ? 'r' : '-',
		   flags & VM_WRITE ? 'w' : '-',
		   flags & VM_EXEC ? 'x' : '-',
		   flags & VM_MAYSHARE ? flags & VM_SHARED ? 'S' : 's' : 'p',
		   ((loff_t)region->vm_pgoff) << PAGE_SHIFT,
		   MAJOR(dev), MINOR(dev), ino, &len);

	if (name) {
		len = 25 + sizeof(void *) * 6 - len;
		if (len < 1)
			len = 1;
		seq_printf(m, "%*c", len, ' ');		
		seq_printf(m, "%s", name);
	}

	seq_putc(m, '\n');
	return 0;
}

/*
 * add a region into the global tree
 */
static void add_mcore_region(struct vm_region *region)
{
	struct vm_region *pregion;
	struct rb_node **p, *parent;

	down_write(&maps_sem);

	parent = NULL;
	p = &mcore_region_tree.rb_node;
	while (*p) {
		parent = *p;
		pregion = rb_entry(parent, struct vm_region, vm_rb);

		if (region->vm_start < pregion->vm_start)
			p = &(*p)->rb_left;
		else if (region->vm_start > pregion->vm_start)
			p = &(*p)->rb_right;
		else if (pregion->vm_start == region->vm_start)
			goto skip;
		else
			BUG();
	}

	rb_link_node(&region->vm_rb, parent, p);
	rb_insert_color(&region->vm_rb, &mcore_region_tree);
skip:
	up_write(&maps_sem);
}

static void register_mcore_region(char *name,
				  unsigned int start,
				  unsigned int size,
				  unsigned int used)
{
	struct vm_region *region;
	char             *region_name;

	region = (struct vm_region *) kzalloc(sizeof(struct vm_region), GFP_KERNEL);
	if (!region)
		return;
	
	region_name = (char *) kzalloc(strlen(name) + 1, GFP_KERNEL);
	if (!region_name)
		return;

	DPRINTK("%s: name = %s, start = 0x%x, size = %d\n",
		__FUNCTION__, name, start, size);

	strcpy(region_name, name);

	/* 
	 * Yes, I know this is ugly but we do not have easy way to get file from 
         * proc_entry
	 */
	region->vm_file  = (struct file *) region_name;
	region->vm_flags = VM_READ | VM_WRITE | VM_MAYSHARE | VM_SHARED;

	/*
	 * If Linux uses this region we need to set it as read-only
	 */
	if (used)
		region->vm_flags &= ~(VM_WRITE);

	region->vm_start = start;
	region->vm_end   = start + size;
	region->vm_pgoff = 0;
	
	add_mcore_region(region);
}

/*
 * display a list of all the MCORE regions
 */
static int maps_list_show(struct seq_file *m, void *_p)
{
	struct rb_node *p = _p;

	return mcore_region_show(m, rb_entry(p, struct vm_region, vm_rb));
}

static void *maps_list_start(struct seq_file *m, loff_t *_pos)
{
	struct rb_node *p;
	loff_t pos = *_pos;

	down_read(&maps_sem);

	for (p = rb_first(&mcore_region_tree); p; p = rb_next(p))
		if (pos-- == 0)
			return p;
	return NULL;
}

static void maps_list_stop(struct seq_file *m, void *v)
{
	up_read(&maps_sem);
}

static void *maps_list_next(struct seq_file *m, void *v, loff_t *pos)
{
	(*pos)++;
	return rb_next((struct rb_node *) v);
}

static const struct seq_operations maps_list_seqop = {
	.start	= maps_list_start,
	.next	= maps_list_next,
	.stop	= maps_list_stop,
	.show	= maps_list_show
};

static int maps_list_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &maps_list_seqop);
}

#if defined(CONFIG_SOC_TMS320C6678) || defined(CONFIG_SOC_TMS320C6670)
static int enable_core_pd(int core_num)
{
        volatile unsigned int *reg;
	unsigned int pd;
	u64 time;

	if ((core_num < 0) || (core_num >= CORE_NUM))
		return -EINVAL;

	/* Enable power domain module */
	pd   = GET_PD(core_num);		
	reg  = (unsigned int*)PSC_PTCMD;
	*reg = (1 << pd);
	time =  sched_clock();

	/* Poll for transition done */
	reg = (unsigned int*)PSC_PTSTAT;
	while ((*reg & (1 << pd))) {
		/* We expect to perform the transition in less than 1ms */
		if ((sched_clock() - time) > 1000000UL) {
			DPRINTK("timeout on transition polling\n");
			return -EBUSY;
		}
	}

	DPRINTK("transition time = %d\n", (int)(sched_clock() - time));
	return 0;
}
#endif

static inline unsigned int psc_mdctl(unsigned int core)
{
	return PSC_MDCTL_CORE0_BASE + PSC_MDCTL_CORE_OFFSET(core);
}

static ssize_t control_proc_write(struct file* file,
				  const char*  buf,
				  size_t       size,
				  loff_t*      ppos)
{
	char cmd, num;
	char bootaddr_str[32];
	unsigned long boot_addr, addr_size;
        volatile unsigned int *reg;
	int res;

	DPRINTK("size=%d\n", size);

	if (!size)
		return 0;

	get_user(cmd, buf);
	get_user(num, buf + 1);

	switch (cmd) {
	case 'b':
	case 'B':
#ifdef CONFIG_SOC_TMS320C6474
		/* First boot for the Faraday case */
		assert_event(INTC_B_EVTPULSE4);
#endif
#if defined(CONFIG_SOC_TMS320C6678) || defined(CONFIG_SOC_TMS320C6670)
        	if (size < 4)
		      return -EFAULT;

		DPRINTK("buf = %s, size = %d:%d\n", buf, size, sizeof (bootaddr_str));

		memset(bootaddr_str, 0, sizeof (bootaddr_str)); 
        	addr_size = size-3;

        	if (copy_from_user(bootaddr_str, (buf + 3), addr_size))    
			return -EFAULT;

		DPRINTK("addr_size = %ld\n", addr_size);
		DPRINTK("bootaddr_str = %s\n", bootaddr_str);

		boot_addr = simple_strtoul(bootaddr_str,
					   (char **)(bootaddr_str + addr_size - 1),
					   16);
		if (boot_addr <= 0) {
			printk(KERN_ERR "Error converting string to integer\n");
			return -EFAULT;
		} else {
	       		DPRINTK("boot_addr = 0x%lx\n", boot_addr);
		}

		/* boot address should be 1KB aligned */
		if (boot_addr & 0x3ff)
			return -EFAULT;
#endif
		if ((num == 0) || (num == 10)) {
			/* Boot all cores */
			int i;
			for (i = 0; i < CORE_NUM; i++)
				if (i != get_coreid()) {
					DPRINTK("booting core %d\n", i);
					*((volatile u32 *) (psc_mdctl(i))) |= 0x100;
			}
#ifdef CONFIG_SOC_TMS320C6472
			/* set BOOT_COMPLETE_STAT */
			*((volatile u32 *) 0x2ab0004 ) = 0x3f;
#endif
#if defined(CONFIG_SOC_TMS320C6678) || defined(CONFIG_SOC_TMS320C6670)
			/* set BOOT_COMPLETE_STAT */
			*((volatile u32 *) DSCR_BOOTCOMPLETE) = 0x3f;
#endif
		} else {
			/* Boot a specific core */
			int core_num;
			core_num = (int) (num - '0');
			if ((core_num >= 0) && 
			    (core_num < CORE_NUM) && 
			    (core_num != get_coreid())) {

#ifdef CONFIG_SOC_TMS320C6472
				*((volatile u32 *) psc_mdctl(core_num)) |= 0x100;
				/* set BOOT_COMPLETE_STAT */
				*((volatile u32 *) 0x2ab0004 ) |= (1 << core_num);
#endif
#if defined(CONFIG_SOC_TMS320C6678) || defined(CONFIG_SOC_TMS320C6670)
			        reg  = (volatile u32 *)DSP_BOOT_ADDR(core_num);	
				*reg = boot_addr;
				
				DPRINTK("booting core %d, boot_addr %p, val read 0x%x\n", core_num, reg, *reg);

				/* Set the boot completed */
				*((volatile u32 *) DSCR_BOOTCOMPLETE) = (1 << core_num);

				/* Take the core out of reset */
				reg  = (unsigned int*) psc_mdctl(core_num);
				*reg = (*reg & ~ MDCTL_NEXT_STATE_MASK) | MDCTL_NEXT_STATE_EN;
				*reg = (*reg & ~ MDCTL_LRSTZ_MASK) | MDCTL_LRSTZ_MASK;

				DPRINTK("reg = %p, val = %x\n", reg, *reg );

				res = enable_core_pd(core_num);
				if (res)
					return res;
#endif
			}

		}
		break;
	case 'r':
	case 'R':
		if ((num == 0) || (num == 10)) {
			/* Reset all cores */
			int i;
			for (i = 0; i < CORE_NUM; i++)
				if (i != get_coreid()) {
					DPRINTK("reseting core %d\n", i);
					*((volatile u32 *) psc_mdctl(i)) &= ~0x100;
				}
		} else {
			/* Reset a specific core */
			int core_num;
			core_num = (int) (num - '0');
			if ((core_num >= 0) && 
			    (core_num < CORE_NUM) && 
			    (core_num != get_coreid())) {
				DPRINTK("reseting core %d\n", core_num);
#ifdef CONFIG_SOC_TMS320C6472
				*((volatile u32 *) psc_mdctl(core_num)) &= ~0x100;
#endif
#if defined(CONFIG_SOC_TMS320C6678) || defined(CONFIG_SOC_TMS320C6670)
				reg  = (unsigned int*) psc_mdctl(core_num);
				*reg = (*reg & ~ MDCTL_NEXT_STATE_MASK) | MDCTL_NEXT_STATE_EN;
				*reg = (*reg & ~ MDCTL_LRSTZ_MASK) | MDCTL_LRSTZ_MASK;

				/* Place the core in reset */
				reg  = (unsigned int*)(psc_mdctl(core_num));
				*reg = (*reg & ~ MDCTL_NEXT_STATE_MASK) | MDCTL_NEXT_STATE_EN;
				*reg = (*reg & ~ MDCTL_LRSTZ_MASK);

				res = enable_core_pd(core_num);
				if (res)
					return res;
#endif
			}
		}
		break;
        };

	*ppos += (loff_t) size;

	return size;
}

static int ram_proc_open(struct inode* inode, struct file*  file)
{
	if (file && file->f_mapping) {
		file->f_mapping->backing_dev_info = &ram_backing_dev_info;
		file->f_mode |= FMODE_WRITE;
	}

	return 0;
}

static loff_t ram_proc_llseek(struct file* file, loff_t off, int whence)
{
	loff_t new;
	struct proc_dir_entry * dp;
	struct inode * inode = file->f_dentry->d_inode;
	struct ram_private_data * data;

	dp   = PDE(inode);
	data = (struct ram_private_data *) dp->data;

	switch (whence) {
	case 0:	 new = off; break;
	case 1:	 new = file->f_pos + off; break;
	case 2:	 new = 1 + off; break;
	default: return -EINVAL;
	}
	
	if (new> (loff_t) data->size)
		return -EINVAL;
	
	return (file->f_pos = new);
}

static ssize_t ram_proc_write(struct file* file,
			      const char*  buf,
			      size_t       size,
			      loff_t*      ppos)
{
	char *src_buf = (char *) buf;
	size_t count = size;
	struct proc_dir_entry * dp;
	struct inode * inode = file->f_dentry->d_inode;
	struct ram_private_data * data;

	dp   = PDE(inode);
	data = (struct ram_private_data *) dp->data;
	
	if (!size)
		return 0;

	/* For the DDR case, avoid to write in the Linux space */
	if ((data->start == RAM_MEMORY_START) && 
	    ((*ppos + count) < (memory_end - RAM_MEMORY_START)))
		return -EACCES;

#ifdef ARCH_HAS_MSM
	/* For the MSM case, avoid to write in the chunk used by Linux */
	if ((data->start == RAM_MSM_CO_BASE) && 
	    ((*ppos + count) < (msm_get_heap() - RAM_MSM_CO_BASE)))
		return -EACCES;
#endif

	/* Do not allow to write in our SRAM (used as caches by Linux) */
	if (data->start == RAM_SRAM_BASE + (get_coreid() * RAM_SRAM_OFFSET))
		return -EACCES;

	/* Eventually truncate the size */
	if ((*ppos + count) > (size_t) data->size)
		count = (size_t) data->size - *ppos;

	/* Because Faraday doesn't include MMU, user buffer is contiguous */
	if (copy_from_user(data->start + (u32) *ppos, src_buf, count))
		return -EFAULT;

	/* Sync cache for DDR case */
	if (data->start == RAM_MEMORY_START)
		L2_cache_block_writeback(data->start + (u32) *ppos,
					 data->start + (u32) *ppos + count);

	*ppos += (u64) count;
	src_buf += count;

	return count;
}

static ssize_t ram_proc_read(struct file* file,
			     char*        buf,
			     size_t       size,
			     loff_t*      ppos)
{
	char * dest_buf = buf;
	size_t count = size;
	struct proc_dir_entry * dp;
	struct inode * inode = file->f_dentry->d_inode;
	struct ram_private_data * data;

	dp   = PDE(inode);
	data = (struct ram_private_data *) dp->data;

	/* Eventually truncate the size */
	if ((*ppos + count) > (size_t) data->size)
		count = (size_t) data->size - *ppos;

	/* Because Faraday doesn't include MMU, user buffer is contiguous */
	if (copy_to_user(dest_buf, data->start + (u32) *ppos, count))
		return -EFAULT;	

	*ppos += (u64) count;
	dest_buf += count;

	return count;
}

static int ram_proc_mmap (struct file *file, struct vm_area_struct *vma)
{
	struct inode * inode = file->f_dentry->d_inode;
	struct proc_dir_entry * dp;
	struct ram_private_data * data;

	dp   = PDE(inode);
	data = (struct ram_private_data *) dp->data;
	
	vma->vm_start = data->start;
	vma->vm_end  += vma->vm_start;

	return 1;
}

static int ram_proc_create(struct proc_dir_entry   *parent, 
			   const  char             *name,
			   struct file_operations  *fops,
			   struct ram_private_data *data)
{
	struct proc_dir_entry *file;

	file = create_proc_entry(name, (S_IFREG|S_IRUGO|S_IWUSR), parent);
	if (!file) {
		printk(KERN_ERR "MCORE: error -- create_proc_entry(%s) failed\n", name);
		return -1;
	}
	
	fops->open      = ram_proc_open;
	fops->read      = ram_proc_read;
	fops->write     = ram_proc_write;
	fops->llseek    = ram_proc_llseek;
	fops->ioctl     = file->proc_fops->ioctl;
	fops->mmap      = ram_proc_mmap;

	fops->get_unmapped_area = get_fb_unmapped_area;

	file->data      = (void*) data;
	file->size      = 0;
	file->proc_fops = fops;

	return 0;
}

static const struct file_operations maps_proc_fops = {
	.open    = maps_list_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release,
};

static int mcore_init(void)
{
	struct proc_dir_entry *mcore;
	struct proc_dir_entry *file;
	unsigned int           n = 0;

	mcore = proc_mkdir("mcore", NULL);
	if (!mcore) {
		printk(KERN_ERR "MCORE: error -- proc_mkdir(/proc/nk) failed\n");
		return -EBUSY;
	}

	/* Create each core entry */
	for (n = 0; n < CORE_NUM; n ++) {
		struct proc_dir_entry   *core;
		struct ram_private_data *data;
		char str[2] = { '0', 0 };
		char name[16];

		str[0] += (char) n;
		core = proc_mkdir(str, mcore);
		if (!core)
			return -EBUSY;

		/* L2 memory */
		data = (struct ram_private_data *) 
			kmalloc(sizeof(struct ram_private_data), GFP_KERNEL);

		data->core_id = n;
		data->start   = RAM_SRAM_BASE + (n * RAM_SRAM_OFFSET);
		data->size    = RAM_SRAM_SIZE;

		if (ram_proc_create(core, "sram", &ram_proc_fops, data))
			return -EBUSY;
		
		/* Disable caching for the L2 regions */
		disable_caching((unsigned int *) data->start,
				(unsigned int *) (data->start + data->size - 1));

		sprintf(name, "SRAM core %d", data->core_id);
		register_mcore_region(name, data->start, data->size, 
				      data->core_id == get_coreid() ? 1 : 0);

		DPRINTK("create SRAM, core=%d, start=0x%x size=0x%x\n",
			data->core_id, data->start, data->size);

#ifdef ARCH_HAS_MSM
		/* MSM memory */
		data = (struct ram_private_data *) 
			kmalloc(sizeof(struct ram_private_data), GFP_KERNEL);

		data->core_id = n;
		data->start   = RAM_MSM_CO_BASE; /* coherent mapping of the MSM */
		data->size    = RAM_MSM_SIZE;

		if (ram_proc_create(core, "msm", &ram_proc_fops, data))
			return -EBUSY;

		/* Look if we use part of the MSM in Linux */
		if (msm_get_heap() > data->start)
			register_mcore_region("MSM", data->start,
					      msm_get_heap() - data->start, 1);
		
		if (msm_get_heap())
			register_mcore_region("MSM", msm_get_heap(),
					      data->size - (msm_get_heap() - data->start), 0);
		else
			register_mcore_region("MSM", data->start, data->size, 0);
		
		DPRINTK("create MSM, core=%d, start=0x%x size=0x%x\n",
		       data->core_id, data->start, data->size);

#endif /* ARCH_HAS_MSM */

		if (n != get_coreid()) {
			data = (struct ram_private_data *) 
				kmalloc(sizeof(struct ram_private_data), GFP_KERNEL);

			data->core_id = n;
			data->start   = RAM_MEMORY_START;
		     
			/* DDR size is board dependent */
			data->size    = BOARD_RAM_SIZE;

			/* Register DDR used by Linux and free DDR */
			register_mcore_region("DDR", data->start,
					      memory_end - data->start, 1);
			if (memory_end < (data->start + data->size - 1))
				register_mcore_region("DDR", memory_end, 
						      data->size - (memory_end - data->start), 0);

			DPRINTK("create DDR, core=%d, start=0x%x size=0x%x\n",
			       data->core_id, data->start, data->size);

			if (ram_proc_create(core, "ddr", &ram_proc_fops, data))
				return -EBUSY;
		}
	}

	/* Initialize mmap() properties */
	memcpy(&ram_backing_dev_info, &default_backing_dev_info, sizeof(struct backing_dev_info));
	ram_backing_dev_info.capabilities |=
		BDI_CAP_MAP_DIRECT | BDI_CAP_READ_MAP | BDI_CAP_WRITE_MAP | BDI_CAP_EXEC_MAP;
	
	/* Create main control file */
	file = create_proc_entry("control", (S_IFREG|S_IRUGO|S_IWUSR), mcore);
	if (!file) {
		printk(KERN_ERR "MCORE: error -- create_proc_entry failed\n");
		return -EBUSY;
	}

	control_proc_fops.read   = file->proc_fops->read;
	control_proc_fops.write  = control_proc_write;
	control_proc_fops.llseek = file->proc_fops->llseek;
	control_proc_fops.ioctl  = file->proc_fops->ioctl;

	file->data      = 0;
	file->size      = 0;
	file->proc_fops = &control_proc_fops;

	/* Create mapping file */
	file = create_proc_entry("maps", (S_IFREG|S_IRUGO|S_IWUSR), mcore);
	if (!file) {
		printk(KERN_ERR "MCORE: error -- create_proc_entry failed\n");
		return -EBUSY;
	}

	file->data      = 0;
	file->size      = 0;
	file->proc_fops = &maps_proc_fops;

	return 0;
}

late_initcall(mcore_init);
