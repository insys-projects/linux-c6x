/*
 *  linux/arch/c6x/kernel/module.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2005, 2009, 2010 Texas Instruments Incorporated
 *  Author: Thomas Charleux (thomas.charleux@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This file handles the architecture-dependent parts of process handling.
 */
#include <linux/moduleloader.h>
#include <linux/elf.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <asm/byteorder.h>

#if 0
#define DEBUGP printk
#else
#define DEBUGP(fmt...)
#endif

void *module_alloc(unsigned long size)
{
	if (size == 0)
		return NULL;
	return vmalloc_exec(size);
}


/* Free memory returned from module_alloc */
void module_free(struct module *mod, void *module_region)
{
	vfree(module_region);
	/* FIXME: If module_region == mod->init_region, trim exception
           table entries. */
}

/*
 * finish loading the module
 */
int module_finalize(const Elf_Ehdr *hdr,
		    const Elf_Shdr *sechdrs,
		    struct module *me)
{
#ifdef CONFIG_TI_C6X_COMPILER
	kfree(me->arch.sh_addr);
	me->arch.sh_addr = NULL;
#endif
	return module_bug_finalize(hdr, sechdrs, me);
}

/*
 * finish clearing the module
 */
void module_arch_cleanup(struct module *mod)
{
	module_bug_cleanup(mod);
}


/* We don't need anything special. */
int module_frob_arch_sections(Elf_Ehdr *hdr,
			      Elf_Shdr *sechdrs,
			      char *secstrings,
			      struct module *mod)
{
#ifdef CONFIG_TI_C6X_COMPILER
	int i, max_alloc = 0;
	Elf_Shdr shdr, *shdrs = (void *)hdr + hdr->e_shoff;

	/* find max SHF_ALLOC section */
	for (i = 1; i < hdr->e_shnum; i++) {
		memcpy(&shdr, &shdrs[i], sizeof(shdr));
		if (shdr.sh_flags & SHF_ALLOC)
			max_alloc = i;
	}

	/* allocate memory for original sh_addr values */
	mod->arch.sh_addr = kzalloc((max_alloc + 1) * sizeof(Elf_Addr *), GFP_KERNEL);
	if (!mod->arch.sh_addr)
		return -ENOMEM;

	/* now get the original sh_addr values */
	for (i = 1; i < hdr->e_shnum; i++) {
		memcpy(&shdr, &shdrs[i], sizeof(shdr));
		if(shdr.sh_flags & SHF_ALLOC)
			mod->arch.sh_addr[i] = shdr.sh_addr;
	}
#endif
	return 0;
}

#define FETCH_PKT(x)  ((x) & 0xFFFFFFE0)


/*
 * apply a REL relocation
 */
int apply_relocate(Elf32_Shdr *sechdrs,
		   const char *strtab,
		   unsigned int symindex,
		   unsigned int relsec,
		   struct module *me)
{
	Elf32_Rel *rel = (void *) sechdrs[relsec].sh_addr;
	Elf_Sym *sym;
	u32 *location, opcode, ep;
	unsigned int i;
	Elf32_Addr v;
	int res;
#ifdef CONFIG_TI_C6X_COMPILER
	Elf_Addr offset = me->arch.sh_addr[sechdrs[relsec].sh_info];
#else
	Elf_Addr offset = 0;
#endif

	DEBUGP("Applying relocate section %u to %u with offset[0x%x]\n", relsec,
	       sechdrs[relsec].sh_info, offset);

	for (i = 0; i < sechdrs[relsec].sh_size / sizeof(*rel); i++) {
		/* This is where to make the change */
		location = (void *)sechdrs[sechdrs[relsec].sh_info].sh_addr
			+ rel[i].r_offset - offset;

		/* this is the execution packet start */
		ep = (u32)location & ~31;

		/* This is the symbol it is referring to.  Note that all
		   undefined symbols have been resolved.  */
		sym = (Elf_Sym *)sechdrs[symindex].sh_addr
			+ ELF32_R_SYM(rel[i].r_info);

		/* this is the adjustment to be made */
#ifdef CONFIG_TI_C6X_COMPILER
		v = sym->st_value - me->arch.sh_addr[sym->st_shndx];
#else
		v = sym->st_value;
#endif

		switch (ELF32_R_TYPE(rel[i].r_info)) {
		case R_C6000_ABS32:
			DEBUGP("REL ABS32: [%p] = 0x%x\n", location, v);
			*location = v;
			break;
		case R_C6000_ABS16:
			DEBUGP("REL ABS16: [%p] = 0x%x\n", location, v);
			*(u16 *)location = v;
			break;
		case R_C6000_ABS8:
			DEBUGP("REL ABS8: [%p] = 0x%x\n", location, v);
			*(u8 *)location = v;
			break;
		case R_C6000_PCR_S21:
			opcode = *location;
			opcode &= ~0x0fffff80;
			opcode |= ((((v - ep) >> 2) & 0x1fffff) << 7);
			DEBUGP("REL PCR_S21[%p] v[0x%x] opcode[0x%x]\n", location, v, opcode);
			*location = opcode;
			break;
		case R_C6000_PCR_S12:
			opcode = *location;
			opcode &= ~0x0fff0000;
			opcode |= ((((v - ep) >> 2) & 0xfff) << 16);
			DEBUGP("REL PCR_S12[%p] v[0x%x] opcode[0x%x]\n", location, v, opcode);
			*location = opcode;
			break;
		case R_C6000_PCR_S10:
			opcode = *location;
			opcode &= ~0x7fe000;
			opcode |= ((((v - ep) >> 2) & 0x3ff) << 13);
			DEBUGP("REL PCR_S10[%p] v[0x%x] opcode[0x%x]\n", location, v, opcode);
			*location = opcode;
			break;
		default:
			printk(KERN_ERR "module %s: Unknown REL relocation: %u\n",
			       me->name, ELF32_R_TYPE(rel[i].r_info));
			return -ENOEXEC;
		}
	}

	return 0;
}

/*
 * apply a RELA relocation
 */
int apply_relocate_add(Elf32_Shdr *sechdrs,
		       const char *strtab,
		       unsigned int symindex,
		       unsigned int relsec,
		       struct module *me)
{
	Elf32_Rela *rel = (void *) sechdrs[relsec].sh_addr;
	Elf_Sym *sym;
	u32 *location, opcode;
	unsigned int i;
	Elf32_Addr v;
	int res;
#ifdef CONFIG_TI_C6X_COMPILER
	Elf_Addr offset = me->arch.sh_addr[sechdrs[relsec].sh_info];
#else
	Elf_Addr offset = 0;
#endif

	DEBUGP("Applying relocate section %u to %u with offset 0x%x\n", relsec,
	       sechdrs[relsec].sh_info, offset);

	for (i = 0; i < sechdrs[relsec].sh_size / sizeof(*rel); i++) {
		/* This is where to make the change */
		location = (void *)sechdrs[sechdrs[relsec].sh_info].sh_addr
			+ rel[i].r_offset - offset;
		/* This is the symbol it is referring to.  Note that all
		   undefined symbols have been resolved.  */
		sym = (Elf_Sym *)sechdrs[symindex].sh_addr
			+ ELF32_R_SYM(rel[i].r_info);

		/* this is the adjustment to be made */
#ifdef CONFIG_TI_C6X_COMPILER
		v = sym->st_value + rel[i].r_addend - me->arch.sh_addr[sym->st_shndx];
#else
		v = sym->st_value + rel[i].r_addend;
#endif

		switch (ELF32_R_TYPE(rel[i].r_info)) {
		case R_C6000_ABS_L16:
			opcode = *location;
			opcode &= ~0x7fff80;
			opcode |= ((v & 0xffff) << 7);
			DEBUGP("RELA ABS_L16[%p] v[0x%x] opcode[0x%x]\n", location, v, opcode);
			*location = opcode;
			break;
		case R_C6000_ABS_H16:
			opcode = *location;
			opcode &= ~0x7fff80;
			opcode |= ((v >> 9) & 0x7fff80);
			DEBUGP("RELA ABS_H16[%p] v[0x%x] opcode[0x%x]\n", location, v, opcode);
			*location = opcode;
			break;
		default:
			printk(KERN_ERR "module %s: Unknown RELA relocation: %u\n",
			       me->name, ELF32_R_TYPE(rel[i].r_info));
			return -ENOEXEC;
		}
	}

	return 0;
}

