/*
 *  linux/arch/c6x/kernel/mcore.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2007, 2009, 2010, 2011 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
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
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/backing-dev.h>
#include <asm/uaccess.h>
#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/machdep.h>
#include <asm/percpu.h>
#include <asm/dscr.h>
#ifdef ARCH_HAS_MSM
#include <asm/msmc.h>
#endif
#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG "MCORE: [%s] " fmt, __FUNCTION__ , ## args)
#else
#define DPRINTK(fmt, args...) 
#endif

static struct file_operations ram_proc_fops;
static struct file_operations control_proc_fops;
static struct backing_dev_info ram_backing_dev_info;

struct ram_private_data {
	u32 core_id;
	u32 start;
	u32 size;
};

extern unsigned int memory_end;

static ssize_t control_proc_write(struct file* file,
				  const char*  buf,
				  size_t       size,
				  loff_t*      ppos)
{
	char cmd, num;
	char bootaddr_str[32];
	unsigned int psc_mdctl = 0;
	unsigned long boot_addr, addr_size, pd;
        volatile unsigned int *reg;

	DPRINTK("size=%d\n", size);

#ifdef CONFIG_SOC_TMS320C6474
	psc_mdctl = PSC_MDCTL3;
#endif
#ifdef CONFIG_SOC_TMS320C6472
	psc_mdctl = PSC_MDCTL0;
#endif
#if defined(CONFIG_SOC_TMS320C6678) || defined(CONFIG_SOC_TMS320C6670)
	psc_mdctl = PSC_CORE0_TIMER0_BASE;
#endif
	DPRINTK("psc_mdctl = 0x%x\n", psc_mdctl);

	if ((!size) || (!psc_mdctl))
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

		boot_addr = simple_strtoul(bootaddr_str, (char **)(bootaddr_str+addr_size-1), 16);
		if (boot_addr <= 0) {
			printk("Error converting string to integer\n");
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
					printk(KERN_INFO "MCORE: booting core %d\n", i);
					*((volatile u32 *) (psc_mdctl) + i) |= 0x100;
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
				*((volatile u32 *) (psc_mdctl) + core_num) |= 0x100;
				/* set BOOT_COMPLETE_STAT */
				*((volatile u32 *) 0x2ab0004 ) |= (1 << core_num);
#endif
#if defined(CONFIG_SOC_TMS320C6678) || defined(CONFIG_SOC_TMS320C6670)
			        reg  = (volatile u32 *)DSP_BOOT_ADDR(core_num);	
				*reg = boot_addr;

				printk(KERN_INFO "MCORE: booting core %d, boot_addr %p, val read 0x%x\n", core_num, reg, *reg);
				/* set BOOT_COMPLETE_STAT */
				*((volatile u32 *) DSCR_BOOTCOMPLETE) = (1 << core_num);

				/* Take the core out of reset */
				reg  = (unsigned int*)(psc_mdctl + (core_num * 4));
				*reg = (*reg & ~ MDCTL_NEXT_STATE_MASK) | MDCTL_NEXT_STATE_EN;
				*reg = (*reg & ~ MDCTL_LRSTZ_MASK) | MDCTL_LRSTZ_MASK;
				DPRINTK("reg = %p, val = %x\n", reg, *reg );

				/* Enable power domain module */
				pd   = GET_PD(core_num);		
				reg  = (unsigned int*)PSC_PTCMD;
				*reg = (1 << pd);

				/* Poll for transition done */
				reg = (unsigned int*)PSC_PTSTAT;
				while ((*reg & (1 << pd)));
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
					printk(KERN_INFO "MCORE: reseting core %d\n", i);
					*((volatile u32 *) (psc_mdctl) + i) &= ~0x100;
				}
		} else {
			/* Reset a specific core */
			int core_num;
			core_num = (int) (num - '0');
			if ((core_num >= 0) && 
			    (core_num < CORE_NUM) && 
			    (core_num != get_coreid())) {
				printk(KERN_INFO "MCORE: reseting core %d\n", core_num);
#ifdef CONFIG_SOC_TMS320C6472
				*((volatile u32 *) (psc_mdctl) + core_num) &= ~0x100;
#endif
#if defined(CONFIG_SOC_TMS320C6678) || defined(CONFIG_SOC_TMS320C6670)
				reg = (unsigned int*)(psc_mdctl + (core_num * 4));
				*reg = (*reg & ~ MDCTL_NEXT_STATE_MASK) | MDCTL_NEXT_STATE_EN;
				*reg = (*reg & ~ MDCTL_LRSTZ_MASK) | MDCTL_LRSTZ_MASK;

				/* Place the core in reset */
				reg = (unsigned int*)(psc_mdctl + (core_num * 4));
				*reg = (*reg & ~ MDCTL_NEXT_STATE_MASK) | MDCTL_NEXT_STATE_EN;
				*reg = (*reg & ~ MDCTL_LRSTZ_MASK);

				/* Enable power domain module */
				pd = GET_PD(core_num);		
				reg = (unsigned int*)PSC_PTCMD;
				*reg = (1 << pd);

				/* Poll for transition done */
				reg = (unsigned int*)PSC_PTSTAT;
				while ((*reg & (1 << pd)));
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

	dp = PDE(inode);
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

	dp = PDE(inode);
	data = (struct ram_private_data *) dp->data;
	
	if (!size)
		return 0;

	/* For the DDR case, avoid to write in the Linux space */
	if ((data->start == RAM_MEMORY_START) && 
	    ((*ppos + count) < (memory_end - RAM_MEMORY_START)))
		return -EACCES;

#ifdef ARCH_HAS_MSM
	if ((data->start == RAM_MSM_CO_BASE) && 
	    ((*ppos + count) < (msm_get_heap() - RAM_MSM_CO_BASE)))
		return -EACCES;
#endif

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

	dp = PDE(inode);
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

	dp = PDE(inode);
	data = (struct ram_private_data *) dp->data;
	
	vma->vm_start = data->start;
	vma->vm_end  += vma->vm_start;

	return 1;
}

static int ram_proc_create(struct proc_dir_entry   *parent, 
			   const char*              name,
			   struct file_operations  *fops,
			   struct ram_private_data *data)
{
	struct proc_dir_entry* file;

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

static int mcore_init(void)
{
	struct proc_dir_entry* mcore;
	struct proc_dir_entry* file;
	unsigned int n = 0;

	mcore = proc_mkdir("mcore", NULL);
	if (!mcore) {
		printk(KERN_ERR "MCORE: error -- proc_mkdir(/proc/nk) failed\n");
		return -EBUSY;
	}

	/* Create each core entry */
	for (n = 0; n < CORE_NUM; n ++) {
		struct proc_dir_entry* core;
		struct ram_private_data * data;
		char str[2] = { '0', 0 };

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

		printk(KERN_INFO "MCORE: create SRAM, core=%d, start=0x%x size=0x%x\n",
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

		printk(KERN_INFO "MCORE: create MSM, core=%d, start=0x%x size=0x%x\n",
		       data->core_id, data->start, data->size);
#endif /* ARCH_HAS_MSM */

		if (n != get_coreid()) {
			data = (struct ram_private_data *) 
				kmalloc(sizeof(struct ram_private_data), GFP_KERNEL);

			data->core_id = n;
			data->start   = RAM_MEMORY_START;
		     
			/* Size is board dependent so set it to the max */
			data->size    = (unsigned int) (0 - data->start);

			printk(KERN_INFO "MCORE: create DDR, core=%d, start=0x%x size=0x%x\n",
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

	return 0;
}

module_init(mcore_init);
