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
#ifdef __TI_EABI__
#include <linux/elf.h>
#else
#include <linux/coff.h>
#endif
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
#ifdef __TI_TOOL_WRAPPER__
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


#ifdef __TI_EABI__
/* We don't need anything special. */
int module_frob_arch_sections(Elf_Ehdr *hdr,
			      Elf_Shdr *sechdrs,
			      char *secstrings,
			      struct module *mod)
{
#ifdef __TI_TOOL_WRAPPER__
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
#ifdef __TI_TOOL_WRAPPER__
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
		v = sym->st_value - me->arch.sh_addr[sym->st_shndx];

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
#ifdef __TI_TOOL_WRAPPER__
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
		v = sym->st_value + rel[i].r_addend - me->arch.sh_addr[sym->st_shndx];

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

#else /* __TI_EABI__ */
/* We don't need anything special. */
int module_frob_arch_sections(COFF_FILHDR *hdr,
			      COFF_SCNHDR *sechdrs,
			      char *secstrings,
			      struct module *mod)
{
	return 0;
}

int apply_relocate(COFF_SCNHDR *sechdrs,
		   const char *strtab,
		   COFF_FILHDR *hdr,
		   unsigned int relsec,
		   struct loaded_sections *lsecs,
		   struct module *me)
{
	unsigned int n_reloc;
	unsigned int inst_addr;
	unsigned int instruction;
	unsigned int label;
	unsigned int reloc_amount;
	COFF_RELOC *creloc;
	COFF_SYMENT *sym;
	int status;
        /*****************************************************************************/
        /* Complex Relocation Expression Stack                                       */
        /*****************************************************************************/
	relocation_stack relstk;
	relstk.stack = NULL;
	relstk.index = -1;
	relstk.size  = 2;

	relstk.stack = (unsigned int *) kmalloc (relstk.size * sizeof(unsigned int), GFP_KERNEL);
	if (relstk.stack == NULL)
		return -ENOMEM;

	DEBUGP("Applying relocate section %s\n",
	       COFF_LONG(sechdrs[relsec].s_name) ? sechdrs[relsec].s_name : strtab + COFF_LONG((sechdrs[relsec].s_name + 4)));

	for(n_reloc = 0; n_reloc < COFF_LONG(sechdrs[relsec].s_nreloc); n_reloc++) {
		creloc = ((void *)hdr) + COFF_LONG(sechdrs[relsec].s_relptr) + (n_reloc * COFF_RELSZ);

		if (COFF_SHORT(creloc->r_type) == R_ABS) break;

		/*
		 * Extract the relocatable field from the object data.
		 */
		inst_addr = COFF_LONG(sechdrs[relsec].s_paddr) + COFF_LONG(creloc->r_vaddr) - COFF_LONG(sechdrs[relsec].s_vaddr);

		sym = NULL;
		if ( COFF_LONG(creloc->r_symndx) != -1 &&
			(COFF_SHORT(creloc->r_type) == RE_PUSH
				|| COFF_SHORT(creloc->r_type) == R_RELLONG
				|| COFF_SHORT(creloc->r_type) == R_C6XLO16
				|| COFF_SHORT(creloc->r_type) == R_C6XHI16))
			sym = ((void *)hdr) + COFF_LONG(hdr->f_symptr) + COFF_LONG(creloc->r_symndx) * COFF_SYMESZ;

		/*------------------------------------------------------------------*/
		/* If this is a relocation expression arithmetic or push            */
		/* instruction, then we can handle it now and move on to the next   */
		/* relocation entry.                                                */
		/*------------------------------------------------------------------*/
		if (ismathrel(COFF_SHORT(creloc->r_type))) { 
			if ((status = coff_rel_math(creloc, &relstk)) < 0)
				return status;
			continue;
		}

		if (ispushrel(COFF_SHORT(creloc->r_type))) {
			if ((status = coff_rel_push(sym, creloc, 0, &relstk)) < 0)
				return status;
			continue;
		}

		/*-----------------------------------------------------------------*/
		/* Handle relocation expression field instructions.                */
		/*-----------------------------------------------------------------*/
		if (isstfldrel(COFF_SHORT(creloc->r_type))) {
			if ((status = coff_rel_stfld((unsigned int *)inst_addr, creloc, &relstk)) < 0)
				return status;
			continue;
		}

		if ( sym ) {
			if ( !COFF_SHORT(sym->e_scnum) )
				reloc_amount = COFF_LONG(sym->e_value);
			else
				reloc_amount = COFF_LONG(sechdrs[COFF_SHORT(sym->e_scnum)-1].s_paddr) - 
					COFF_LONG(sechdrs[COFF_SHORT(sym->e_scnum)-1].s_vaddr);
		}
		else
			reloc_amount = me->module_core + lsecs[relsec].new_vaddr - COFF_LONG(sechdrs[relsec].s_vaddr);

		/*
		 * Modify the field based on the relocation type.
		 */
		switch (COFF_SHORT(creloc->r_type)) {

		case R_RELLONG:
			/*
			 * Normal relocations :
			 * add in the relocation amount.
			 */
			label = (unsigned int) *((unsigned int *) inst_addr);
			label += reloc_amount;
			*((unsigned int *) inst_addr) = label;
			break;

		case R_C6XLO16:
			/*
			 * MVK 16-bit Constant.
			 * Calculate address by masking
			 * off the opcode and OR in the lower 16-bit
			 * constant + the amount
			 */
			
			instruction = (unsigned int) *((unsigned int *) inst_addr);
			label = (instruction & 0x007FFF80) >> 7;
			label += reloc_amount;
			instruction &= 0xFF80007F ;
			instruction |= (label & 0x0000FFFF) << 7;
			*((unsigned int *) inst_addr) = instruction;
			break;

		case R_C6XHI16:
			/*
			 * MVK 16-bit Constant.  Calculate address by masking
			 * off the opcode and OR in the lower 16-bit constant 
			 * + the amount
			 */
			instruction = (unsigned int) *((unsigned int *) inst_addr);
			label = ((instruction & 0x007FFF80) << 9) | 
				(COFF_SHORT(creloc->r_disp) & 0x0000FFFF);
			label += reloc_amount;
			instruction &= 0xFF80007F;
			instruction |= (label & 0xFFFF0000) >> 9;
			*((unsigned int *)inst_addr) = instruction;
      			break;

		case R_C6XBASE:
		case R_C6XPCR21:
			break;

			/*
			 * All other cases are treated as default for which future 
			 * extension are requested.
			 */
		default:
			printk(KERN_ERR "module %s: Unknown relocation: %u\n",
			       me->name, COFF_SHORT(creloc->r_type));
			return -ENOEXEC;
		}		
	}

	if (relstk.index != -1) {
		printk(KERN_ALERT "Complex relocation stack should be empty!\n");
		return -ENOEXEC;		
	}

	kfree(relstk.stack);
	return 0;
}

#define ALIGN_COFF_MASK 0x7
#define ALIGN_COFF_PTR(ptr) \
   ((unsigned *)(((unsigned)ptr + ALIGN_COFF_MASK) & ~ALIGN_COFF_MASK))
#define ALIGN_COFF_VAL(val) \
   ((((unsigned)val + ALIGN_COFF_MASK) & ~ALIGN_COFF_MASK))

void handle_cinit(COFF_SCNHDR *sechdrs,
		  unsigned int cinitsec)
{
	const unsigned int *recptr = COFF_LONG(sechdrs[cinitsec].s_paddr);
	int length, cinit_size = COFF_LONG(sechdrs[cinitsec].s_size);

	while( cinit_size > 0 ) {
		length = *recptr++;
		cinit_size -= 4;

		if (length < 0){
			printk(KERN_WARNING "negative length in .cinit section\n");
			cinit_size -= ALIGN_COFF_VAL(recptr) - (unsigned)recptr;
			recptr = ALIGN_COFF_PTR(recptr);
		} else {
			char *to    = *recptr++;
			char *from  = (char *)recptr;

			memcpy( to, from, length);
					
			from += length;
			cinit_size -= ALIGN_COFF_VAL(from) - (unsigned)from + length + 4;
			recptr = ALIGN_COFF_PTR(from);
		}
	}	  
}
#endif /* __TI_EABI__ */
