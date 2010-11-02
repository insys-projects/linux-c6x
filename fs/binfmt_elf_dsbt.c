/* binfmt_elf_dsbt.c: DSBT ELF binary format
 *
 * Copyright (C) 2010 Texas Instruments Incorporated
 * Written by Mark Salter (msalter@redhat.com)
 * Derived from binfmt_elf_fdpic.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>

#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/binfmts.h>
#include <linux/string.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/security.h>
#include <linux/highmem.h>
#include <linux/highuid.h>
#include <linux/personality.h>
#include <linux/ptrace.h>
#include <linux/init.h>
#include <linux/elf.h>
#include <linux/elf-dsbt.h>
#include <linux/elfcore.h>
#include <linux/coredump.h>

#include <asm/uaccess.h>
#include <asm/param.h>
#include <asm/pgalloc.h>

typedef char *elf_caddr_t;

#if 0
#define kdebug(fmt, ...) printk("DSBT: "fmt"\n" ,##__VA_ARGS__ )
#else
#define kdebug(fmt, ...) do {} while(0)
#endif

#if 0
#define kdcore(fmt, ...) printk("DSBT "fmt"\n" ,##__VA_ARGS__ )
#else
#define kdcore(fmt, ...) do {} while(0)
#endif

#if ELF_EXEC_PAGESIZE > PAGE_SIZE
#define ELF_MIN_ALIGN	ELF_EXEC_PAGESIZE
#else
#define ELF_MIN_ALIGN	PAGE_SIZE
#endif

#if ELF_CLASS == ELFCLASS32
#define dt_tag_t Elf32_Sword
#define elf_off  Elf32_Off
#else
#define dt_tag_t Elf64_Sxword
#define elf_off  Elf64_Off
#endif


#define ELF_PAGESTART(_v) ((_v) & ~(unsigned long)(ELF_MIN_ALIGN-1))
#define ELF_PAGEOFFSET(_v) ((_v) & (ELF_MIN_ALIGN-1))
#define ELF_PAGEALIGN(_v) (((_v) + ELF_MIN_ALIGN - 1) & ~(ELF_MIN_ALIGN - 1))

#define BAD_ADDR(x) ((unsigned long)(x) >= TASK_SIZE)

MODULE_LICENSE("GPL");

static int load_elf_dsbt_binary(struct linux_binprm *, struct pt_regs *);
static int elf_dsbt_fetch_phdrs(struct elf_dsbt_params *, struct file *);
static int elf_dsbt_map_file(struct elf_dsbt_params *, struct file *,
			     struct mm_struct *, const char *, const char *);

static int create_elf_dsbt_tables(struct linux_binprm *, struct mm_struct *,
				   struct elf_dsbt_params *,
				   struct elf_dsbt_params *);

static int elf_dsbt_transfer_args_to_stack(struct linux_binprm *,
					    unsigned long *);

static struct linux_binfmt elf_dsbt_format = {
	.module		= THIS_MODULE,
	.load_binary	= load_elf_dsbt_binary,
	.min_coredump	= ELF_EXEC_PAGESIZE,
};

static int __init init_elf_dsbt_binfmt(void)
{
	return register_binfmt(&elf_dsbt_format);
}

static void __exit exit_elf_dsbt_binfmt(void)
{
	unregister_binfmt(&elf_dsbt_format);
}

core_initcall(init_elf_dsbt_binfmt);
module_exit(exit_elf_dsbt_binfmt);

static int is_elf_dsbt(struct elfhdr *hdr, struct file *file)
{
	if (memcmp(hdr->e_ident, ELFMAG, SELFMAG) != 0)
		return 0;
	if (hdr->e_type != ET_EXEC && hdr->e_type != ET_DYN)
		return 0;
	if (!elf_check_arch(hdr) || !elf_check_dsbt(hdr))
		return 0;
	if (!file->f_op || !file->f_op->mmap)
		return 0;
	return 1;
}

/*****************************************************************************/
/*
 * read the program headers table into memory
 */
static int elf_dsbt_fetch_phdrs(struct elf_dsbt_params *params,
				 struct file *file)
{
	struct elf32_phdr *phdr;
	unsigned long size;
	int retval, loop;

	if (params->hdr.e_phentsize != sizeof(struct elf_phdr))
		return -ENOMEM;
	if (params->hdr.e_phnum > 65536U / sizeof(struct elf_phdr))
		return -ENOMEM;

	size = params->hdr.e_phnum * sizeof(struct elf_phdr);
	params->phdrs = kmalloc(size, GFP_KERNEL);
	if (!params->phdrs)
		return -ENOMEM;

	retval = kernel_read(file, params->hdr.e_phoff,
			     (char *) params->phdrs, size);
	if (unlikely(retval != size))
		return retval < 0 ? retval : -ENOEXEC;

	/* determine stack size for this binary */
	phdr = params->phdrs;
	for (loop = 0; loop < params->hdr.e_phnum; loop++, phdr++) {
		if (phdr->p_type != PT_GNU_STACK)
			continue;

		params->stack_size = phdr->p_memsz;
		break;
	}

	return 0;
}

/*****************************************************************************/
/*
 * read the section headers table into memory
 */
static int elf_dsbt_fetch_shdrs(struct elf_dsbt_params *params,
				 struct file *file)
{
	unsigned long size;
	int retval;

	if (params->hdr.e_shentsize != sizeof(struct elf_shdr))
		return -ENOMEM;
	if (params->hdr.e_shnum > 65536U / sizeof(struct elf_shdr))
		return -ENOMEM;

	size = params->hdr.e_shnum * sizeof(struct elf_shdr);
	params->shdrs = kmalloc(size, GFP_KERNEL);
	if (!params->shdrs)
		return -ENOMEM;

	retval = kernel_read(file, params->hdr.e_shoff,
			     (char *) params->shdrs, size);
	if (unlikely(retval != size))
		return retval < 0 ? retval : -ENOEXEC;

	return 0;
}

/*****************************************************************************/
/*
 * read the .dynamic section into memory
 */
#define  N_EXTRA_DTAGS 5
static int elf_dsbt_fetch_dynamic(struct elf_dsbt_params *params,
				  struct file *file)
{
	struct elf_shdr *shdr;
	int i, retval;

	shdr = &params->shdrs[1];
	for (i = 1; i < params->hdr.e_shnum; i++, shdr++) {
		if (shdr->sh_type != SHT_DYNAMIC)
			continue;

		/*
		 * add a little extra room in case we need
		 * to add some tags
		 */
		params->dynsize = shdr->sh_size + (sizeof(elf_dyn) * N_EXTRA_DTAGS);

		params->dynamic = kmalloc(params->dynsize, GFP_KERNEL);
		if (!params->dynamic)
			return -ENOMEM;
		
		retval = kernel_read(file, shdr->sh_offset,
			     (char *) params->dynamic, shdr->sh_size);

		if (unlikely(retval != shdr->sh_size))
			return retval < 0 ? retval : -ENOEXEC;
		break;
	}
	return 0;
}

/*****************************************************************************/
/*
 * search for dynamic section tag value
 */
static elf_dyn *find_dynamic_tag(struct elf_dsbt_params *params,
				 dt_tag_t tag)
{
	elf_dyn *d = params->dynamic;

	while (d->d_tag) {
		if (d->d_tag == tag)
			return d;
		++d;
	}
	return NULL;
}

/*****************************************************************************/
/*
 * search for section by section offset
 */
static struct elf_shdr *find_section_by_offset(struct elf_dsbt_params *params,
					elf_off offset)
{
	struct elf_shdr *s;
	int i;

	s = &params->shdrs[1];
	for (i = 1; i < params->hdr.e_shnum; i++, s++) {
		if (offset < s->sh_offset)
			continue;
		if (offset >= (s->sh_offset + s->sh_size))
			continue;
		kdebug("found unmapped section %d for 0x%x bytes at offset[%x]",
		       i, s->sh_size, offset);
		return s;
	}
	return NULL;
}

/*****************************************************************************/
/*
 * load a DSBT binary into various bits of memory
 */
static int load_elf_dsbt_binary(struct linux_binprm *bprm,
				 struct pt_regs *regs)
{
	struct elf_dsbt_params exec_params, interp_params;
	struct elf_phdr *phdr;
	unsigned long stack_size, entryaddr;
#ifdef ELF_DSBT_PLAT_INIT
	unsigned long dynaddr, dsbt_table;
#endif
	unsigned long stack_prot;
	struct file *interpreter = NULL; /* to shut gcc up */
	char *interpreter_name = NULL;
	int executable_stack;
	int retval, i;

	kdebug("____ LOAD  pid[%d] ____", current->pid);

	memset(&exec_params, 0, sizeof(exec_params));
	memset(&interp_params, 0, sizeof(interp_params));

	exec_params.hdr = *(struct elfhdr *) bprm->buf;
	exec_params.flags = ELF_DSBT_FLAG_EXECUTABLE | ELF_DSBT_FLAG_PRESENT;

	/* check that this is a binary we know how to deal with */
	retval = -ENOEXEC;
	if (!is_elf_dsbt(&exec_params.hdr, bprm->file)) {
		kdebug("not DSBT binary!");
		goto error;
	}

	/* read the program header table */
	retval = elf_dsbt_fetch_phdrs(&exec_params, bprm->file);
	if (retval < 0) {
		kdebug("can't get phdrs!");
		goto error;
	}

	/* read the section header table */
	retval = elf_dsbt_fetch_shdrs(&exec_params, bprm->file);
	if (retval < 0) {
		kdebug("can't get shdrs!");
		goto error;
	}

	/* read the dynamic section */
	retval = elf_dsbt_fetch_dynamic(&exec_params, bprm->file);
	if (retval < 0) {
		kdebug("can't get dynamic section!");
		goto error;
	}

	/* scan for a program header that specifies an interpreter */
	phdr = exec_params.phdrs;

	for (i = 0; i < exec_params.hdr.e_phnum; i++, phdr++) {
		switch (phdr->p_type) {
		case PT_INTERP:
			kdebug("found PT_INTERP");
			retval = -ENOMEM;
			if (phdr->p_filesz > PATH_MAX)
				goto error;
			retval = -ENOENT;
			if (phdr->p_filesz < 2)
				goto error;

			/* read the name of the interpreter into memory */
			interpreter_name = kmalloc(phdr->p_filesz, GFP_KERNEL);
			if (!interpreter_name)
				goto error;

			retval = kernel_read(bprm->file,
					     phdr->p_offset,
					     interpreter_name,
					     phdr->p_filesz);
			if (unlikely(retval != phdr->p_filesz)) {
				if (retval >= 0)
					retval = -ENOEXEC;
				goto error;
			}

			retval = -ENOENT;
			if (interpreter_name[phdr->p_filesz - 1] != '\0')
				goto error;

			kdebug("Using ELF interpreter %s", interpreter_name);

			/* replace the program with the interpreter */
			interpreter = open_exec(interpreter_name);
			retval = PTR_ERR(interpreter);
			if (IS_ERR(interpreter)) {
				interpreter = NULL;
				goto error;
			}

			/*
			 * If the binary is not readable then enforce
			 * mm->dumpable = 0 regardless of the interpreter's
			 * permissions.
			 */
			if (file_permission(interpreter, MAY_READ) < 0)
				bprm->interp_flags |= BINPRM_FLAGS_ENFORCE_NONDUMP;

			retval = kernel_read(interpreter, 0, bprm->buf,
					     BINPRM_BUF_SIZE);
			if (unlikely(retval != BINPRM_BUF_SIZE)) {
				if (retval >= 0)
					retval = -ENOEXEC;
				goto error;
			}

			interp_params.hdr = *((struct elfhdr *) bprm->buf);
			break;

		case PT_LOAD:
			kdebug("found PT_LOAD");
			break;
		}

	}
	if (!interpreter_name) {
		interpreter_name = kmalloc(32, GFP_KERNEL);
		retval = -ENOMEM;
		if (!interpreter_name)
			goto error;
		strcpy(interpreter_name, "/lib/ld-uClibc.so.0");
		kdebug("did NOT found PT_INTERP");
		kdebug("Using ELF interpreter %s", interpreter_name);

		/* replace the program with the interpreter */
		retval = -ENOENT;
		interpreter = open_exec(interpreter_name);
		retval = PTR_ERR(interpreter);
		if (IS_ERR(interpreter)) {
			interpreter = NULL;
			goto error;
		}

		/*
		 * If the binary is not readable then enforce
		 * mm->dumpable = 0 regardless of the interpreter's
		 * permissions.
		 */
		if (file_permission(interpreter, MAY_READ) < 0)
			bprm->interp_flags |= BINPRM_FLAGS_ENFORCE_NONDUMP;

		retval = kernel_read(interpreter, 0, bprm->buf,
				     BINPRM_BUF_SIZE);
		if (unlikely(retval != BINPRM_BUF_SIZE)) {
			if (retval >= 0)
				retval = -ENOEXEC;
			goto error;
		}

		interp_params.hdr = *((struct elfhdr *) bprm->buf);
	}

	/* perform insanity checks on the interpreter */
	if (interpreter_name) {
		retval = -ELIBBAD;
		if (!is_elf_dsbt(&interp_params.hdr, interpreter))
			goto error;

		interp_params.flags = ELF_DSBT_FLAG_PRESENT;

		/* read the interpreter's program header table */
		retval = elf_dsbt_fetch_phdrs(&interp_params, interpreter);
		if (retval < 0)
			goto error;

		/* read the interpreter's section header table */
		retval = elf_dsbt_fetch_shdrs(&interp_params, interpreter);
		if (retval < 0)
			goto error;

		/* read the interpreter's dynamic section */
		retval = elf_dsbt_fetch_dynamic(&interp_params, interpreter);
		if (retval < 0) {
			kdebug("can't get dynamic section!");
			goto error;
		}
	}

	executable_stack = EXSTACK_DEFAULT;

	stack_size = interp_params.stack_size;

	retval = -ENOEXEC;
	if (stack_size == 0) {
#if 0
		kdebug("zero stack size");
		goto error;
#else
		stack_size = 0x8000;
#endif
	}

	/* flush all traces of the currently running executable */
	retval = flush_old_exec(bprm);
	if (retval) {
		kdebug("flush_old_exec returned: %d", retval);
		goto error;
	}

	/* there's now no turning back... the old userspace image is dead,
	 * defunct, deceased, etc. after this point we have to exit via
	 * error_kill */
	set_personality(PER_LINUX);
	if (elf_read_implies_exec(&exec_params.hdr, executable_stack))
		current->personality |= READ_IMPLIES_EXEC;

	setup_new_exec(bprm);

	set_binfmt(&elf_dsbt_format);

	current->mm->start_code = 0;
	current->mm->end_code = 0;
	current->mm->start_stack = 0;
	current->mm->start_data = 0;
	current->mm->end_data = 0;

	current->flags &= ~PF_FORKNOEXEC;

	/* load the executable and interpreter into memory */
	retval = elf_dsbt_map_file(&exec_params, bprm->file, current->mm,
				   "executable", interpreter_name);
	if (retval < 0) {
		kdebug("elf_dsbt_map_file: returned %d", retval);
		goto error_kill;
	}

	if (interpreter_name) {
		retval = elf_dsbt_map_file(&interp_params, interpreter,
					   current->mm, "interpreter", NULL);
		if (retval < 0) {
			printk(KERN_ERR "Unable to load interpreter");
			goto error_kill;
		}

		allow_write_access(interpreter);
		fput(interpreter);
		interpreter = NULL;
	}

	/* create a stack and brk area big enough for everyone
	 * - the brk heap starts at the bottom and works up
	 * - the stack starts at the top and works down
	 */
	stack_size = (stack_size + PAGE_SIZE - 1) & PAGE_MASK;
	if (stack_size < PAGE_SIZE * 2)
		stack_size = PAGE_SIZE * 2;

	stack_prot = PROT_READ | PROT_WRITE;
	if (executable_stack == EXSTACK_ENABLE_X ||
	    (executable_stack == EXSTACK_DEFAULT && VM_STACK_FLAGS & VM_EXEC))
		stack_prot |= PROT_EXEC;

	down_write(&current->mm->mmap_sem);
	current->mm->start_brk = do_mmap(NULL, 0, stack_size, stack_prot,
					 MAP_PRIVATE | MAP_ANONYMOUS |
					 MAP_UNINITIALIZED | MAP_GROWSDOWN,
					 0);

	if (IS_ERR_VALUE(current->mm->start_brk)) {
		up_write(&current->mm->mmap_sem);
		retval = current->mm->start_brk;
		current->mm->start_brk = 0;
		kdebug("do_mmap failed");
		goto error_kill;
	}

	up_write(&current->mm->mmap_sem);

	current->mm->brk = current->mm->start_brk;
	current->mm->context.end_brk = current->mm->start_brk;
	current->mm->context.end_brk +=
		(stack_size > PAGE_SIZE) ? (stack_size - PAGE_SIZE) : 0;
	current->mm->start_stack = current->mm->start_brk + stack_size;

	install_exec_creds(bprm);
	current->flags &= ~PF_FORKNOEXEC;
	if (create_elf_dsbt_tables(bprm, current->mm,
				   &exec_params, &interp_params) < 0) {
		kdebug("create_eld_dsbt_tables failed");
		goto error_kill;
	}

	kdebug("- start_code  %lx", current->mm->start_code);
	kdebug("- end_code    %lx", current->mm->end_code);
	kdebug("- start_data  %lx", current->mm->start_data);
	kdebug("- end_data    %lx", current->mm->end_data);
	kdebug("- start_brk   %lx", current->mm->start_brk);
	kdebug("- brk         %lx", current->mm->brk);
	kdebug("- start_stack %lx", current->mm->start_stack);

#ifdef ELF_DSBT_PLAT_INIT
	/*
	 * The ABI may specify that certain registers be set up in special
	 * ways. This macro performs whatever initialization to the regs
	 * structure is required.
	 */
	if (interp_params.dynamic_addr) {
		dynaddr = interp_params.dynamic_addr;
		dsbt_table = (unsigned long)interp_params.loadmap->dsbt_table;
	} else {
		dynaddr = exec_params.dynamic_addr;
		dsbt_table = (unsigned long)exec_params.loadmap->dsbt_table;
	}

	kdebug("exec_params.map_addr[%x]", exec_params.map_addr);
	kdebug("interp_params.map_addr[%x]", interp_params.map_addr);
	ELF_DSBT_PLAT_INIT(regs,
			   exec_params.map_addr,
			   interp_params.map_addr,
			   dynaddr, dsbt_table);
#endif

	/* everything is now ready... get the userspace context ready to roll */
	if (interp_params.entry_addr)
		entryaddr = interp_params.entry_addr;
	else
		entryaddr = exec_params.entry_addr;

	start_thread(regs, entryaddr, current->mm->start_stack);

	retval = 0;
error:
	kdebug("load returning: %d", retval);
	if (interpreter) {
		allow_write_access(interpreter);
		fput(interpreter);
	}
	kfree(interpreter_name);
	kfree(exec_params.phdrs);
	kfree(exec_params.shdrs);
	kfree(interp_params.phdrs);
	kfree(interp_params.shdrs);
	kfree(exec_params.dynamic);
	kfree(interp_params.dynamic);
	kfree(exec_params.extra_segs);
	kfree(interp_params.extra_segs);
	return retval;

	/* unrecoverable error - kill the process */
error_kill:
	send_sig(SIGSEGV, current, 0);
	goto error;

}

/*****************************************************************************/

#ifndef ELF_BASE_PLATFORM
/*
 * AT_BASE_PLATFORM indicates the "real" hardware/microarchitecture.
 * If the arch defines ELF_BASE_PLATFORM (in asm/elf.h), the value
 * will be copied to the user stack in the same manner as AT_PLATFORM.
 */
#define ELF_BASE_PLATFORM NULL
#endif

/*
 * present useful information to the program by shovelling it onto the new
 * process's stack
 */
static int create_elf_dsbt_tables(struct linux_binprm *bprm,
				   struct mm_struct *mm,
				   struct elf_dsbt_params *exec_params,
				   struct elf_dsbt_params *interp_params)
{
	const struct cred *cred = current_cred();
	unsigned long sp, csp, nitems;
	elf_caddr_t __user *argv, *envp;
	size_t platform_len = 0, len;
	char *k_platform, *k_base_platform;
	char __user *u_platform, *u_base_platform, *p;
	long hwcap;
	int loop;
	int nr;	/* reset for each csp adjustment */

	sp = mm->start_stack;

	/* stack the program arguments and environment */
	if (elf_dsbt_transfer_args_to_stack(bprm, &sp) < 0)
		return -EFAULT;

	hwcap = ELF_HWCAP;

	/*
	 * If this architecture has a platform capability string, copy it
	 * to userspace.  In some cases (Sparc), this info is impossible
	 * for userspace to get any other way, in others (i386) it is
	 * merely difficult.
	 */
	k_platform = ELF_PLATFORM;
	u_platform = NULL;

	if (k_platform) {
		platform_len = strlen(k_platform) + 1;
		sp -= platform_len;
		u_platform = (char __user *) sp;
		if (__copy_to_user(u_platform, k_platform, platform_len) != 0)
			return -EFAULT;
	}

	/*
	 * If this architecture has a "base" platform capability
	 * string, copy it to userspace.
	 */
	k_base_platform = ELF_BASE_PLATFORM;
	u_base_platform = NULL;

	if (k_base_platform) {
		platform_len = strlen(k_base_platform) + 1;
		sp -= platform_len;
		u_base_platform = (char __user *) sp;
		if (__copy_to_user(u_base_platform, k_base_platform, platform_len) != 0)
			return -EFAULT;
	}

	sp &= ~7UL;

	/* stack the load map(s) */
	len = sizeof(struct elf32_dsbt_loadmap);
	len += sizeof(struct elf32_dsbt_loadseg) * exec_params->loadmap->nsegs;
	sp = (sp - len) & ~7UL;
	exec_params->map_addr = sp;
	kdebug("exec_params->map_addr[%x]", sp);

	if (copy_to_user((void __user *) sp, exec_params->loadmap, len) != 0)
		return -EFAULT;

	current->mm->context.exec_dsbt_loadmap = (unsigned long) sp;

	if (interp_params->loadmap) {
		len = sizeof(struct elf32_dsbt_loadmap);
		len += sizeof(struct elf32_dsbt_loadseg) *
			interp_params->loadmap->nsegs;
		sp = (sp - len) & ~7UL;
		interp_params->map_addr = sp;
		kdebug("interp_params->map_addr[%x]", sp);
		if (copy_to_user((void __user *) sp, interp_params->loadmap,
				 len) != 0)
			return -EFAULT;

		current->mm->context.interp_dsbt_loadmap = (unsigned long) sp;
	}

	/* force 16 byte _final_ alignment here for generality */
#define DLINFO_ITEMS 15

	nitems = 1 + DLINFO_ITEMS + (k_platform ? 1 : 0) +
		(k_base_platform ? 1 : 0) + AT_VECTOR_SIZE_ARCH;

	if (bprm->interp_flags & BINPRM_FLAGS_EXECFD)
		nitems++;

	csp = sp;
	sp -= nitems * 2 * sizeof(unsigned long);
	sp -= (bprm->envc + 1) * sizeof(char *);	/* envv[] */
	sp -= (bprm->argc + 1) * sizeof(char *);	/* argv[] */
	sp -= 1 * sizeof(unsigned long);		/* argc */

	csp -= sp & 15UL;
	sp -= sp & 15UL;

	/* put the ELF interpreter info on the stack */
#define NEW_AUX_ENT(id, val)						\
	do {								\
		struct { unsigned long _id, _val; } __user *ent;	\
									\
		ent = (void __user *) csp;				\
		__put_user((id), &ent[nr]._id);				\
		__put_user((val), &ent[nr]._val);			\
		nr++;							\
	} while (0)

	nr = 0;
	csp -= 2 * sizeof(unsigned long);
	NEW_AUX_ENT(AT_NULL, 0);
	if (k_platform) {
		nr = 0;
		csp -= 2 * sizeof(unsigned long);
		NEW_AUX_ENT(AT_PLATFORM,
			    (elf_addr_t) (unsigned long) u_platform);
	}

	if (k_base_platform) {
		nr = 0;
		csp -= 2 * sizeof(unsigned long);
		NEW_AUX_ENT(AT_BASE_PLATFORM,
			    (elf_addr_t) (unsigned long) u_base_platform);
	}

	if (bprm->interp_flags & BINPRM_FLAGS_EXECFD) {
		nr = 0;
		csp -= 2 * sizeof(unsigned long);
		NEW_AUX_ENT(AT_EXECFD, bprm->interp_data);
	}

	nr = 0;
	csp -= DLINFO_ITEMS * 2 * sizeof(unsigned long);
	NEW_AUX_ENT(AT_HWCAP,	hwcap);
	NEW_AUX_ENT(AT_PAGESZ,	PAGE_SIZE);
	NEW_AUX_ENT(AT_CLKTCK,	CLOCKS_PER_SEC);
	NEW_AUX_ENT(AT_PHDR,	exec_params->ph_addr);
	NEW_AUX_ENT(AT_PHENT,	sizeof(struct elf_phdr));
	NEW_AUX_ENT(AT_PHNUM,	exec_params->hdr.e_phnum);
	NEW_AUX_ENT(AT_BASE,	interp_params->elfhdr_addr);
	NEW_AUX_ENT(AT_FLAGS,	0);
	NEW_AUX_ENT(AT_ENTRY,	exec_params->entry_addr);
	NEW_AUX_ENT(AT_UID,	(elf_addr_t) cred->uid);
	NEW_AUX_ENT(AT_EUID,	(elf_addr_t) cred->euid);
	NEW_AUX_ENT(AT_GID,	(elf_addr_t) cred->gid);
	NEW_AUX_ENT(AT_EGID,	(elf_addr_t) cred->egid);
	NEW_AUX_ENT(AT_SECURE,	security_bprm_secureexec(bprm));
	NEW_AUX_ENT(AT_EXECFN,	bprm->exec);

	kdebug("AUX entry start: %x]", csp);
#ifdef ARCH_DLINFO
	nr = 0;
	csp -= AT_VECTOR_SIZE_ARCH * 2 * sizeof(unsigned long);

	/* ARCH_DLINFO must come last so platform specific code can enforce
	 * special alignment requirements on the AUXV if necessary (eg. PPC).
	 */
	ARCH_DLINFO;
#endif
#undef NEW_AUX_ENT

	/* allocate room for argv[] and envv[] */
	csp -= (bprm->envc + 1) * sizeof(elf_caddr_t);
	envp = (elf_caddr_t __user *) csp;
	csp -= (bprm->argc + 1) * sizeof(elf_caddr_t);
	argv = (elf_caddr_t __user *) csp;

	kdebug("argv[%x] envp[%x]\n", argv, envp);

	/* stack argc */
	csp -= sizeof(unsigned long);
	__put_user(bprm->argc, (unsigned long __user *) csp);

	BUG_ON(csp != sp);

	/* fill in the argv[] array */
	current->mm->arg_start = current->mm->start_stack -
		(MAX_ARG_PAGES * PAGE_SIZE - bprm->p);

	p = (char __user *) current->mm->arg_start;
	for (loop = bprm->argc; loop > 0; loop--) {
		__put_user((elf_caddr_t) p, argv++);
		len = strnlen_user(p, MAX_ARG_STRLEN);
		if (!len || len > MAX_ARG_STRLEN)
			return -EINVAL;
		p += len;
	}
	__put_user(NULL, argv);
	current->mm->arg_end = (unsigned long) p;

	/* fill in the envv[] array */
	current->mm->env_start = (unsigned long) p;
	for (loop = bprm->envc; loop > 0; loop--) {
		__put_user((elf_caddr_t)(unsigned long) p, envp++);
		len = strnlen_user(p, MAX_ARG_STRLEN);
		if (!len || len > MAX_ARG_STRLEN)
			return -EINVAL;
		p += len;
	}
	__put_user(NULL, envp);
	current->mm->env_end = (unsigned long) p;

	mm->start_stack = (unsigned long) sp;
	return 0;
}

/*****************************************************************************/
/*
 * transfer the program arguments and environment from the holding pages onto
 * the stack
 */
static int elf_dsbt_transfer_args_to_stack(struct linux_binprm *bprm,
					    unsigned long *_sp)
{
	unsigned long index, stop, sp;
	char *src;
	int ret = 0;

	stop = bprm->p >> PAGE_SHIFT;
	sp = *_sp;

	for (index = MAX_ARG_PAGES - 1; index >= stop; index--) {
		src = kmap(bprm->page[index]);
		sp -= PAGE_SIZE;
		if (copy_to_user((void *) sp, src, PAGE_SIZE) != 0)
			ret = -EFAULT;
		kunmap(bprm->page[index]);
		if (ret < 0)
			goto out;
	}

	*_sp = (*_sp - (MAX_ARG_PAGES * PAGE_SIZE - bprm->p)) & ~15;

out:
	return ret;
}

/*****************************************************************************/
/*
 * map a single segment
 */
static unsigned long elf_dsbt_map(struct file *filep, unsigned long addr,
		struct elf32_dsbt_seg *seg, int prot, int type)
{
	unsigned long map_addr;
	unsigned long size = seg->p_memsz + ELF_PAGEOFFSET(seg->p_vaddr);
	unsigned long off = seg->p_offset - ELF_PAGEOFFSET(seg->p_vaddr);
	addr = ELF_PAGESTART(addr);
	size = ELF_PAGEALIGN(size);

	/* mmap() will return -EINVAL if given a zero size, but a
	 * segment with zero filesize is perfectly valid */
	if (!size)
		return addr;

	down_write(&current->mm->mmap_sem);
	map_addr = do_mmap(filep, addr, size, prot, type, off);
	up_write(&current->mm->mmap_sem);
	return(map_addr);
}

/*****************************************************************************/
/* 
 * Early (v7.2.0 alpha) TI linker does not put dynamic sections in a LOAD segment.
 * Here, we find try to figure out which sections should be loaded but are not.
 * Then we can form a phantom load segment and load them.
 *
 * Yes, this is work the static linker should be doing. And we have to duplicate
 * it in the runtime linker as well.
 */

static unsigned long elf_dsbt_map_anon_mem(void *mem, int size, int offset)
{
	unsigned long map_addr;
	int msize = ELF_PAGEALIGN(size);

	if (!size)
		return -EINVAL;

	down_write(&current->mm->mmap_sem);
	map_addr = do_mmap(NULL, 0, msize, PROT_READ | PROT_WRITE, MAP_PRIVATE, 0);
	if (BAD_ADDR(map_addr))
		kdebug("mmap anon for offset %x failed!", offset);
	else {
		kdebug("mmap'ing: 0x%x bytes of anon mem [%x] to [%x]", size, mem, map_addr);
		copy_to_user(map_addr + offset, mem, size);
	}

	up_write(&current->mm->mmap_sem);
	return(map_addr);
}


static const dt_tag_t tags_to_check[] = {
	DT_HASH, DT_STRTAB, DT_SYMTAB, DT_RELA,
	DT_NULL
};

struct unmapped_sects {
	int num;
	struct {
		unsigned long *valp;
		struct elf_shdr *shdr;
	} sections[0];
};

static int find_unmapped_sections(struct elf_dsbt_params *params, struct unmapped_sects **result)
{
	elf_dyn *d;
	struct elf_shdr *s;
	const dt_tag_t *t;
	struct elf32_dsbt_seg *p;
	struct unmapped_sects *usects;

	usects = kmalloc(sizeof(*usects) + (8 * ARRAY_SIZE(tags_to_check)), GFP_KERNEL);
	if (usects == NULL)
		return -ENOMEM;

	usects->num = 0;
	for (t = &tags_to_check[0]; *t != DT_NULL; t++) {
		d = find_dynamic_tag(params, *t);
		if (!d || d->d_un.d_ptr == 0)
			continue;

		/* look for this offset in code segment */
		p = &params->code_seg;
		if (p->p_offset <= d->d_un.d_ptr &&
		    (p->p_offset + p->p_filesz) > d->d_un.d_ptr)
			continue;
		p = &params->data_seg;
		if (p->p_offset <= d->d_un.d_ptr &&
                   (p->p_offset + p->p_filesz) > d->d_un.d_ptr)
			continue;

		s = find_section_by_offset(params, d->d_un.d_ptr);
		if (s == NULL) {
			kdebug("couldn't find section for offset 0x%x", d->d_un.d_ptr);
			kfree(usects);
			return -EINVAL;
		}
		if (s->sh_size) {
			usects->sections[usects->num].valp = (unsigned long *)&d->d_un.d_ptr;
			usects->sections[usects->num++].shdr = s;
		}
	}
	*result = usects;
	return 0;
}

int do_linker_workarounds(struct file *filep, struct elf_dsbt_params *params, const char *interp)
{
	struct unmapped_sects *us;
	struct elf_shdr *s;
	struct elf32_phdr *new_phdrs, *ph;
	struct elf32_dsbt_seg *segs, *seg;
	int i, ret, num_new, num_old, phsize;
	int ehdr_unmapped = 0;
	unsigned long vaddr, addr, size;

	ret = find_unmapped_sections(params, &us);
	if (ret < 0)
		return ret;

	num_new = us->num;

	/* was elf header mapped */
	if (params->code_seg.p_offset != 0 && params->data_seg.p_offset != 0) {
		ehdr_unmapped = 1;
		++num_new;
	}

	if (!num_new) {
		/*
		 * We're good to go. Assumption is that if all of the segments
		 * referenced from dynamic tags are mapped, everything else is
		 * ok with the elf file.
		 */
		kfree(us);
		return 0;
	}

	/* add new program headers and new .dynamic section */
	num_new += 2;

	num_old = params->hdr.e_phnum;
	params->hdr.e_phnum += (num_new + (interp?1:0));

	phsize = params->hdr.e_phnum * sizeof(*new_phdrs);
	new_phdrs = kmalloc(phsize, GFP_KERNEL);
	if (new_phdrs == NULL)
		return -ENOMEM;

	memcpy(new_phdrs, params->phdrs, params->hdr.e_phnum * sizeof(struct elf_phdr));
	segs = kmalloc(num_new * sizeof(struct elf32_dsbt_seg), GFP_KERNEL);
	if (segs == NULL)
		return -ENOMEM;

	params->extra_segs = segs;
	params->num_new = num_new;

	seg = segs;

	/* Find start of unused vaddr space to use for fake segments */
	vaddr = params->code_seg.p_vaddr + params->code_seg.p_memsz;
	if (vaddr < (params->data_seg.p_vaddr + params->data_seg.p_memsz))
		vaddr = params->data_seg.p_vaddr + params->data_seg.p_memsz;
	vaddr = ELF_PAGEALIGN(vaddr);

	ph = &new_phdrs[num_old];

	/* map elf header, if needed */
	if (ehdr_unmapped) {
		ph->p_type = PT_LOAD;
		ph->p_offset = 0;
		ph->p_vaddr = ph->p_paddr = vaddr;
		ph->p_filesz = ph->p_memsz = sizeof(struct elfhdr);
		ph->p_flags = PF_R | PF_W;
		ph->p_align = 1;

		addr = elf_dsbt_map_anon_mem(&params->hdr, ph->p_memsz, 0);
		if (BAD_ADDR(addr))
		    return (int) addr;

		params->elfhdr_addr = addr;

		seg->addr = addr;
		seg->p_vaddr = vaddr;
		seg->p_offset = 0;
		seg->p_filesz = seg->p_memsz = sizeof(struct elfhdr);

		vaddr = ELF_PAGEALIGN(vaddr + ph->p_memsz);

		++ph;
		++seg;
	}

	/* map unmapped segments */
	for (i = 0; i < us->num; i++, s++) {
		char *tmp;

		s = us->sections[i].shdr;

		ph->p_type = PT_LOAD;
		ph->p_offset = 0;
		ph->p_vaddr = ph->p_paddr = vaddr;
		ph->p_filesz = ph->p_memsz = s->sh_size;
		ph->p_flags = PF_R | PF_W;
		ph->p_align = 1;

		tmp = kmalloc(s->sh_size, GFP_KERNEL);
		if (tmp == NULL)
			return -ENOMEM;

		ret = kernel_read(filep, s->sh_offset, tmp, s->sh_size);
		if (ret < 0)
			return ret;

		addr = elf_dsbt_map_anon_mem(tmp, s->sh_size, 0);
		if (BAD_ADDR(addr))
		    return (int) addr;

		*us->sections[i].valp = vaddr;

		kfree(tmp);

		seg->addr = addr;
		seg->p_vaddr = vaddr;
		seg->p_offset = 0;
		seg->p_filesz = seg->p_memsz = s->sh_size;

		vaddr = ELF_PAGEALIGN(vaddr + s->sh_size);

		++ph;
		++seg;
	}
	kfree(us);

	/* map dynamic section (and interp if needed) */
	size = params->dynsize + (interp ? strlen(interp)+1 : 0);
	addr = elf_dsbt_map_anon_mem(params->dynamic, size, 0);
	for (i = 0; i < params->hdr.e_phnum; i++)
		if (new_phdrs[i].p_type == PT_DYNAMIC) {
			new_phdrs[i].p_vaddr = vaddr;
			break;
		}
	params->dynamic_addr = addr;
	ph->p_type = PT_LOAD;
	ph->p_offset = 0;
	ph->p_vaddr = ph->p_paddr = vaddr;
	ph->p_filesz = ph->p_memsz = size;
	ph->p_flags = PF_R | PF_W;
	ph->p_align = 1;
	seg->addr = addr;
	seg->p_vaddr = vaddr;
	seg->p_offset = 0;
	seg->p_filesz = seg->p_memsz = size;
	++seg;
	++ph;

	if (interp) {
		ph->p_type = PT_INTERP;
		ph->p_offset = params->dynsize;
		ph->p_vaddr = ph->p_paddr = vaddr + params->dynsize;
		ph->p_filesz = ph->p_memsz = strlen(interp) + 1;
		ph->p_flags = PF_R;
		ph->p_align = 1;
		strcpy((char *)addr + params->dynsize, interp);
		++ph;
	}

	vaddr = ELF_PAGEALIGN(vaddr + params->dynsize);

	/* map the new program headers */
	ph->p_type = PT_PHDR;
	ph->p_offset = 0;
	ph->p_vaddr = ph->p_paddr = vaddr;
	ph->p_filesz = ph->p_memsz = phsize;
	ph->p_flags = PF_R | PF_W;
	ph->p_align = 1;
	
	addr = elf_dsbt_map_anon_mem(new_phdrs, phsize, 0);
	if (BAD_ADDR(addr))
		return (int) addr;

	seg->addr = addr;
	seg->p_vaddr = vaddr;
	seg->p_offset = 0;
	seg->p_filesz = seg->p_memsz = phsize;

	params->ph_addr = addr;

	kfree(new_phdrs);

	for (i = 0, ph = (struct elf32_phdr *)addr; i < params->hdr.e_phnum; i++, ph++) {
		kdebug("phdr%d: type[%d] vaddr[0x%x] memsize[0x%x]", 
		       i, ph->p_type, ph->p_vaddr, ph->p_memsz);
	}

	return num_new;
}



/*****************************************************************************/
/*
 * load the appropriate binary image (executable or interpreter) into memory
 * - we assume no MMU is available
 * - we assume that there is one LOADable segment with execute flags
 * - we assume that the LOADABLE segments without execute flags must be
 *   mapped together, but not necessarily contiguous with the execute
 *   segments.
 * - we assume R/O executable segments are shareable
 * - TI linker may not include .dynamic or the elf header in a loadable segment
 */
static int elf_dsbt_map_file(struct elf_dsbt_params *params,
			      struct file *file,
			      struct mm_struct *mm,
			      const char *what,
			      const char *interp_name)
{
#define MAX_DSEGS 16
	struct elf32_phdr *phdr, *code_phdr = NULL;
	struct elf32_phdr *base_phdr = NULL;
	struct elf32_phdr *top_phdr = NULL;
	struct elf32_phdr *dyn_phdr = NULL;
	struct elf32_phdr *top_contents_phdr = NULL;
	struct elf32_phdr *dsegs[MAX_DSEGS];
	struct elf32_dsbt_loadmap *loadmap;
	struct elf32_dsbt_loadseg *seg;
	elf_dyn *dyn;
	unsigned long mapped_addr, bss_len, dseg_offset = 0, offset;
	unsigned long top_vaddr,  top_offset;
	int loop, ret, flags, ndseg;

	phdr = params->phdrs;
	ndseg = 0;
	for (loop = 0; loop < params->hdr.e_phnum; loop++, phdr++) {
		if (phdr->p_type == PT_DYNAMIC)
			dyn_phdr = phdr;

		if (phdr->p_type != PT_LOAD)
			continue;

		kdebug("elf_dsbt_map_file: [LOAD] va=%lx of=%lx fs=%lx ms=%lx flags=%lx",
		       (unsigned long) phdr->p_vaddr,
		       (unsigned long) phdr->p_offset,
		       (unsigned long) phdr->p_filesz,
		       (unsigned long) phdr->p_memsz,
		       (unsigned long) phdr->p_flags);

		if (ELF_PAGEOFFSET(phdr->p_offset) != ELF_PAGEOFFSET(phdr->p_vaddr)) {
			kdebug("Bad ELF file!");
			return -EINVAL;
		}

		if ((phdr->p_flags & (PF_X|PF_W)) == PF_X) {
			/*
			 * There should be only one RO executable segment.
			 */
			if (code_phdr) {
				kdebug("more than one code segment!");
				return -EINVAL;
			}
			code_phdr = phdr;
		} else {
			/*
			 * There may be multiple non-executable segments, but they
			 * must be contiguous so we can map them in one map. We assume
			 * that any zero filesz segments come after any segments with
			 * file contents.
			 */
			if (ndseg == MAX_DSEGS) {
				kdebug("too many RW segments!");
				return -EINVAL;
			}
			dsegs[ndseg++] = phdr;

			offset = phdr->p_vaddr - phdr->p_offset;
			if (ndseg == 1)
				dseg_offset = offset;
			else if (dseg_offset != offset) {
 				kdebug("non-contiguous RW segment layout!");
				return -EINVAL;
			}
			if (!base_phdr) {
				base_phdr = top_phdr = phdr;
				if (base_phdr->p_filesz)
					top_contents_phdr = phdr;
				continue;
			}

			if (base_phdr->p_vaddr > phdr->p_vaddr)
				base_phdr = phdr;

			if ((top_phdr->p_vaddr + top_phdr->p_memsz) < (phdr->p_vaddr + phdr->p_memsz)) {
				top_phdr = phdr;
				if (phdr->p_filesz)
					top_contents_phdr = phdr;
			}
		}
	}
	if (!code_phdr) {
		kdebug("no executable segment!");
		return -EINVAL;
	}
	if (!base_phdr) {
		kdebug("no RW segment!");
		return -EINVAL;
	}

	params->code_seg.p_vaddr  = code_phdr->p_vaddr;
	params->code_seg.p_offset = code_phdr->p_offset;
	params->code_seg.p_memsz  = code_phdr->p_memsz;
	params->code_seg.p_filesz = code_phdr->p_filesz;

	top_vaddr = top_phdr->p_vaddr + top_phdr->p_memsz;
	top_offset = top_phdr->p_offset + top_phdr->p_filesz;

	params->data_seg.p_vaddr = base_phdr->p_vaddr;
	params->data_seg.p_offset = base_phdr->p_offset;
	params->data_seg.p_memsz = params->data_seg.p_filesz =
		top_phdr->p_vaddr + top_phdr->p_memsz - base_phdr->p_vaddr;

	kdebug("base_phdr: vaddr[%x] offset[%x] memsz[%x] filesz[%x]",
	       base_phdr->p_vaddr, base_phdr->p_offset, base_phdr->p_memsz, base_phdr->p_filesz);

	kdebug("top_phdr: vaddr[%x] offset[%x] memsz[%x] filesz[%x]",
	       top_phdr->p_vaddr, top_phdr->p_offset, top_phdr->p_memsz, top_phdr->p_filesz);

	kdebug("top_contents: vaddr[%x] offset[%x] memsz[%x] filesz[%x]",
	       top_contents_phdr->p_vaddr, top_contents_phdr->p_offset, top_contents_phdr->p_memsz, top_contents_phdr->p_filesz);

	if (top_contents_phdr)
		params->data_seg.p_filesz = 
			top_contents_phdr->p_vaddr + top_contents_phdr->p_filesz - base_phdr->p_vaddr;
	else
		params->data_seg.p_filesz = 0;

	kdebug("code_seg: vaddr[%x] offset[%x] memsz[%x] filesz[%x]!",
	       code_phdr->p_vaddr, code_phdr->p_offset, code_phdr->p_memsz, code_phdr->p_filesz);

#if 0
	/* map code segment */
	flags = MAP_PRIVATE | MAP_DENYWRITE;
	if (params->flags & ELF_DSBT_FLAG_EXECUTABLE)
		flags |= MAP_EXECUTABLE;

	mapped_addr = elf_dsbt_map(file, params->code_seg.p_vaddr, &params->code_seg,
				   PROT_READ | PROT_EXEC, flags);
	if (BAD_ADDR(mapped_addr))
		return (int) mapped_addr;

	params->code_seg.addr = mapped_addr + ELF_PAGEOFFSET(params->code_seg.p_vaddr);

	kdebug("mapped code seg from %08x to %08x", params->code_seg.p_vaddr, params->code_seg.addr);

	kdebug("data_seg: vaddr[%x] offset[%x] memsz[%x] filesz[%x]!",
	       params->data_seg.p_vaddr, params->data_seg.p_offset, params->data_seg.p_memsz, params->data_seg.p_filesz);

	/* map data segment */
	mapped_addr = elf_dsbt_map(file, params->data_seg.p_vaddr, &params->data_seg,
				   PROT_READ | PROT_WRITE, MAP_PRIVATE);
	if (BAD_ADDR(mapped_addr)) {
		do_munmap(current->mm, mapped_addr, params->data_seg.p_memsz);
		return (int) mapped_addr;
	}
	params->data_seg.addr = mapped_addr + ELF_PAGEOFFSET(params->data_seg.p_vaddr);

	kdebug("mapped data seg from %08x to %08x", params->data_seg.p_vaddr, params->data_seg.addr);

#else
	/* this goes away with a tool fix for far DP-relative accesses to const data */
	{
		struct elf32_dsbt_seg xseg;

		xseg.p_vaddr = params->code_seg.p_vaddr;
		xseg.p_offset = params->code_seg.p_offset;
		xseg.p_filesz = params->data_seg.p_offset + params->data_seg.p_filesz - xseg.p_offset;
		xseg.p_memsz = params->data_seg.p_offset + params->data_seg.p_memsz - xseg.p_offset;

		mapped_addr = elf_dsbt_map(file, xseg.p_vaddr, &xseg,
					   PROT_READ | PROT_WRITE | PROT_EXEC, MAP_PRIVATE);
	}
	if (BAD_ADDR(mapped_addr))
		return (int) mapped_addr;

	params->code_seg.addr = mapped_addr + ELF_PAGEOFFSET(params->code_seg.p_vaddr);
	params->data_seg.addr = params->code_seg.addr + (params->data_seg.p_vaddr - params->code_seg.p_vaddr);
#endif

	/* clear any bss area */
	bss_len = params->data_seg.p_memsz - params->data_seg.p_filesz;
	if (bss_len) {
		unsigned long bss_start;

		bss_start = params->data_seg.addr + params->data_seg.p_filesz;

		kdebug("clearing %d bytes of bss at %08x", bss_len, bss_start);
		ret = clear_user((void *)bss_start, bss_len);
		if (ret)
			return ret;
	}

	/* fetch dsbt info from dynamic section */
	dyn = find_dynamic_tag(params, DT_C6000_DSBT_BASE);
	if (!dyn) {
		kdebug("DSBT BASE not found");
		return -EINVAL;
	}
	/* relocate it */
	params->dsbt_base = (dyn->d_un.d_ptr - params->data_seg.p_vaddr) + params->data_seg.addr;

	dyn = find_dynamic_tag(params, DT_C6000_DSBT_SIZE);
	if (!dyn) {
		kdebug("DSBT SIZE not found");
		return -EINVAL;
	}
	params->dsbt_size = dyn->d_un.d_ptr;

	dyn = find_dynamic_tag(params, DT_C6000_DSBT_INDEX);
	if (!dyn) {
		kdebug("DSBT INDEX not found");
		return -EINVAL;
	}
	params->dsbt_index = dyn->d_un.d_ptr;
	
	/* 
	 * Now we need to deal with some problems with early TI linker where parts of the
	 * elf image which need to be loaded but are not in any loadable segment.
	 */
	ret = do_linker_workarounds(file, params, interp_name);
	if (ret < 0)
		return ret;

	if (ret == 0) {
		/* no linker workarounds needed, so do this the normal way */

		/* determine where the elf header has wound up if mapped */
		if (params->code_seg.p_offset == 0)
			params->elfhdr_addr = params->code_seg.addr;
		else if (params->data_seg.p_offset == 0)
			params->elfhdr_addr = params->data_seg.addr;
		else {
			/* elf header has not been mapped */
			kdebug("elf header not mapped!");
			return -EINVAL;
		}

		/* determine where the program header table has wound up if mapped */
		if (params->hdr.e_phoff >= params->code_seg.p_offset &&
		    params->hdr.e_phoff < params->code_seg.p_offset + params->code_seg.p_memsz)
			params->ph_addr = params->code_seg.addr +
				(params->hdr.e_phoff - params->code_seg.p_offset);
		else if (params->hdr.e_phoff >= params->data_seg.p_offset &&
			 params->hdr.e_phoff < params->data_seg.p_offset + params->data_seg.p_memsz)
			params->ph_addr = params->data_seg.addr +
				(params->hdr.e_phoff - params->data_seg.p_offset);
		else {
			/* program header has not been mapped */
			kdebug("program header not mapped!");
			return -EINVAL;
		}

		/* determine where the dynamic section has wound up if there is one */
		if (dyn_phdr) {
			if (dyn_phdr->p_offset >= params->code_seg.p_offset &&
			    dyn_phdr->p_offset < params->code_seg.p_offset + params->code_seg.p_memsz)
				params->dynamic_addr = params->code_seg.addr +
					(dyn_phdr->p_offset - params->code_seg.p_offset);
			else if (dyn_phdr->p_offset >= params->data_seg.p_offset &&
				 dyn_phdr->p_offset < params->data_seg.p_offset + params->data_seg.p_memsz)
				params->dynamic_addr = params->data_seg.addr +
					(dyn_phdr->p_offset - params->data_seg.p_offset);
			else {
				/* dynamic section has not been mapped */
				kdebug("dynamic section not mapped!");
				return -EINVAL;
			}
		}
	}

	loadmap = kmalloc(sizeof(*loadmap) + (ret + 2) * sizeof(struct elf32_dsbt_loadseg),
			  GFP_KERNEL);
	if (!loadmap)
		return -ENOMEM;

	params->loadmap = loadmap;

	loadmap->version = ELF_DSBT_LOADMAP_VERSION;
	loadmap->dsbt_table = (unsigned *)params->dsbt_base;
	loadmap->dsbt_size = params->dsbt_size;
	loadmap->dsbt_index = params->dsbt_index;

	loadmap->nsegs = ret + 2;
	
	seg = loadmap->segs;

	seg->addr = params->code_seg.addr;
	seg->p_vaddr = params->code_seg.p_vaddr;
	seg->p_memsz = params->code_seg.p_memsz;
	++seg;
	seg->addr = params->data_seg.addr;
	seg->p_vaddr = params->data_seg.p_vaddr;
	seg->p_memsz = params->data_seg.p_memsz;
	++seg;

	for (loop = 0; loop < ret; loop++, seg++) {
		seg->addr = params->extra_segs[loop].addr;
		seg->p_vaddr = params->extra_segs[loop].p_vaddr;
		seg->p_memsz = params->extra_segs[loop].p_memsz;
	}
	
	/* map the entry point */
	if (params->hdr.e_entry)
		params->entry_addr =
			(params->hdr.e_entry - params->code_seg.p_vaddr) + params->code_seg.addr;

	if (mm) {
		mm->start_code = params->code_seg.addr;
		mm->end_code = params->code_seg.addr + params->code_seg.p_memsz;

		mm->start_data = params->data_seg.addr;
		mm->end_data = params->data_seg.addr + params->data_seg.p_memsz;
	}

	kdebug("Mapped Object [%s]:", what);
	kdebug("- elfhdr   : %lx", params->elfhdr_addr);
	kdebug("- PHDR[]   : %lx", params->ph_addr);
	kdebug("- entry    : %lx", params->entry_addr);
	kdebug("- DYNAMIC[]: %lx", params->dynamic_addr);
	kdebug("- CODE     : %lx", params->code_seg.addr);
	kdebug("- DATA     : %lx", params->data_seg.addr);
	kdebug("- DSBT TBL : %lx", loadmap->dsbt_table);
	kdebug("- DSBT SZ  : %d", loadmap->dsbt_size);
	kdebug("- DSBT IDX : %d", loadmap->dsbt_index);

#if 1
	seg = loadmap->segs;
	for (loop = 0; loop < loadmap->nsegs; loop++, seg++) {
		kdebug("SEG%d vaddr[%x] mapped 0x%x bytes to %x",
		       loop, seg->p_vaddr, seg->p_memsz, seg->addr);
	}
#endif

	return 0;
}


