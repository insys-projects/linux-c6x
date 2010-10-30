/*
 *  linux/arch/c6x/kernel/mcore.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2007, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@virtuallogix.com)
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
	unsigned int psc_mdctl = 0;

#ifdef CONFIG_SOC_TMS320C6474
	psc_mdctl = PSC_MDCTL3;
#endif
#ifdef CONFIG_SOC_TMS320C6472
	psc_mdctl = PSC_MDCTL0;
#endif

	if ((!size) || (!psc_mdctl))
		return 0;

	get_user(cmd, buf);
	get_user(num, buf + 1);

	switch (cmd) {
	case 'b':
	case 'B':
#ifdef CONFIG_SOC_TMS320C6474
		/* First boot for the Faraday case */
	        assert_event(IRQ_B_EVTPULSE4);
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
		} else {
			/* Boot a specific core */
			int core_num;
			core_num = (int) (num - '0');
			if ((core_num >= 0) && 
			    (core_num < CORE_NUM) && 
			    (core_num != get_coreid())) {
				printk(KERN_INFO "MCORE: booting core %d\n", core_num);
				*((volatile u32 *) (psc_mdctl) + core_num) |= 0x100;
#ifdef CONFIG_SOC_TMS320C6472
				/* set BOOT_COMPLETE_STAT */
				*((volatile u32 *) 0x2ab0004 ) |= (1 << core_num);
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
				*((volatile u32 *) (psc_mdctl) + core_num) &= ~0x100;
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
	int res;
	char * src_buf = buf;
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
	int res;
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
	unsigned int core_id;
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

	file->data      = (u32) data;
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

		data = (struct ram_private_data *) 
			kmalloc(sizeof(struct ram_private_data), GFP_KERNEL);

		data->core_id = n;
		data->start   = RAM_SRAM_BASE + (n * RAM_SRAM_OFFSET);
		data->size    = RAM_SRAM_SIZE;

		if (ram_proc_create(core, "sram", &ram_proc_fops, data))
			return -EBUSY;
		
		/* Disable caching for the L2 regions */
		disable_caching(data->start, data->start + data->size - 1);

		printk(KERN_INFO "MCORE: create SRAM, core=%d, start=0x%x size=0x%x\n",
		       data->core_id, data->start, data->size);

		if (n != get_coreid()) {
			unsigned int start;

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
