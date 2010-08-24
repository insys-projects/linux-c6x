/* elf-dsbt.h: DSBT ELF load map
 *
 * Copyright (C) 2010 Texas Instruments Incorporated
 * Written by Mark Salter <msalter@redhat.com>
 * Based on elf-fdpic.h
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifndef _LINUX_ELF_DSBT_H
#define _LINUX_ELF_DSBT_H

#include <linux/elf.h>

#if ELF_CLASS == ELFCLASS32
#define elf_dyn Elf32_Dyn
#else
#define elf_dyn Elf64_Dyn
#endif

#define ELF_DSBT_LOADMAP_VERSION	0x0000

/* This data structure represents a PT_LOAD segment.  */
struct elf32_dsbt_loadseg
{
	/* Core address to which the segment is mapped.  */
	Elf32_Addr addr;
	/* VMA recorded in the program header.  */
	Elf32_Addr p_vaddr;
	/* Size of this segment in memory.  */
	Elf32_Word p_memsz;
};

struct elf32_dsbt_loadmap {
	/* Protocol version number, must be zero.  */
	Elf32_Word version;

	/* Pointer to DSBT */
	unsigned   *dsbt_table;
	unsigned   dsbt_size;
	unsigned   dsbt_index;

	/* number of segments */
	Elf32_Word nsegs;

	/* The actual memory map.  */
	struct elf32_dsbt_loadseg segs[0];
};

/* internal segment mappings for ELF DSBT loader */
struct elf32_dsbt_seg {
	Elf32_Addr	addr;		/* core address to which mapped */
	Elf32_Addr	p_vaddr;	/* VMA recorded in file */
	Elf32_Off	p_offset;	/* file offset */
	Elf32_Word	p_filesz;	/* initialized data in file */
	Elf32_Word	p_memsz;	/* allocation size recorded in file */
};

/*
 * binfmt binary parameters structure
 */
struct elf_dsbt_params {
	struct elfhdr			hdr;		/* ref copy of ELF header */
	struct elf_phdr			*phdrs;		/* ref copy of PT_PHDR table */
	struct elf_shdr			*shdrs;	        /* ref copy of PT_SHDR table */
	elf_dyn				*dynamic;	/* ref copy of .dynamic section */
	int				dynsize;	/* size of .dynamic section */
	struct elf32_dsbt_seg           code_seg;       /* mapping for code segment */
	struct elf32_dsbt_seg           data_seg;       /* mapping for data segment */
	struct elf32_dsbt_seg           *extra_segs;    /* mapping for extra segments */
	struct elf32_dsbt_loadmap       *loadmap;       /* loadmap to be passed to userspace */
	int				num_new;
	unsigned long			elfhdr_addr;	/* mapped ELF header user address */
	unsigned long			ph_addr;	/* mapped PT_PHDR user address */
	unsigned long			map_addr;	/* mapped loadmap user address */
	unsigned long			entry_addr;	/* mapped entry user address */
	unsigned long			stack_size;	/* stack size requested (PT_GNU_STACK) */
	unsigned long			dynamic_addr;	/* mapped PT_DYNAMIC user address */
	unsigned long			flags;
	unsigned long			dsbt_base;
	unsigned int			dsbt_size;
	unsigned int			dsbt_index;

#define ELF_DSBT_FLAG_PRESENT		0x00000001	/* T if this object is present */
#define ELF_DSBT_FLAG_EXECUTABLE	0x00000002	/* T if this object is the executable */
};

#endif /* _LINUX_ELF_DSBT_H */
