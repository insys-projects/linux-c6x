/*
 *  fs/binfmt_coff.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2007, 2009, 2010 Texas Instruments Incorporated
 *  Authors: Al Longyear (longyear@sii.com)
 *           Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *           Gregory Thiemonge (gregory.thiemonge@virtuallogix.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  These are the functions used to load COFF IBCS style executables.
 *  Information on COFF format may be obtained in either the Intel Binary
 *  Compatibility Specification 2 or O'Rilley's book on COFF. The shared
 *  libraries are defined only the in the Intel book.
 *
 *  This file is based upon code written by Eric Youngdale for the ELF object
 *  file format.
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/a.out.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/binfmts.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/ptrace.h>
#include <linux/coff.h>
#include <linux/slab.h>
#include <linux/personality.h>
#include <linux/file.h>
#include <linux/unistd.h>

#ifdef CONFIG_BINFMT_COFF_INFLATE
#include <linux/zlib.h>
#endif /* CONFIG_BINFMT_COFF_INFLATE */

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/pgalloc.h>
#include <asm/cacheflush.h>

MODULE_DESCRIPTION("Support for the COFF binary format");
MODULE_AUTHOR("Al Longyear, Aurelien Jacquiot, Gregory Thiemonge");
MODULE_LICENSE("GPL");

#undef COFF_TRACE
#undef COFF_TRACE_VERBOUS

#define ALIGN_FETCH_MASK 0xFF
#define ALIGN_FETCH_PTR(ptr) \
   ((unsigned int)(((unsigned int)ptr + ALIGN_FETCH_MASK) & ~ALIGN_FETCH_MASK))

#define ALIGN_COFF_MASK 0x7
#define ALIGN_COFF_PTR(ptr) \
   ((unsigned *)(((unsigned)ptr + ALIGN_COFF_MASK) & ~ALIGN_COFF_MASK))

#define COFF_READ(file, buf, size, pos) \
  (file)->f_op->read((file), \
                           (char *) (buf), \
                           (unsigned long) (size), \
                           (loff_t *) (pos))
          
#define position_max(a,b) ((a)>(b)?(a):(b))

static int	coff_load_binary(struct linux_binprm *, struct pt_regs *);
static int	coff_parse_comments(struct file *, COFF_SCNHDR *, long *);
struct linux_binfmt coff_format = {
	.module = THIS_MODULE,
	.load_binary = coff_load_binary,
	.min_coredump = PAGE_SIZE,
};

#define COFF_BUFFER_SIZE (COFF_RELSZ * 50 * 200)

#define COFF_SECTION_INITIALIZER { 0 , 0 ,0 }

typedef struct coff_section {
	long	scnptr;
	long	size;
	long	vaddr;
} coff_section;

typedef struct coff_clue {
	short	terminal;	/* non-zero to stop parsing with this entry */
	short	len;		/* negative number uses strstr for lookup   */
	char	*text;		/* text to search for			    */
	u_long	mask_and, mask_or;
} coff_clue;

/*
 * The following table gives clues to the "personality" of the executable
 * which we hope to find in the .comment sections of the binary.
 * The set here may not be comprehensive even for those systems listed.
 * Use 'mcs -p' to list the .comments sections of a binary to see what
 * clues might be there. Or use 'strings' if you don't have mcs.
 */
static coff_clue coff_clues[] = {
	/*
	 * Wyse Unix V/386 3.2.1[A].
	 */
	{1, 36, "@(#) UNIX System V/386 Release 3.2.1", 0, PER_WYSEV386},

	/*
	 * SCO Unix V 3.2, 3.2.2, 3.2.4, 3.2.4.2 etc.
	 */
	{1, 23, "@(#) crt1.s 1.8 89/05/30", 0, PER_SCOSVR3},
	{1, 16, "@(#)SCO UNIX 3.2", 0, PER_SCOSVR3},
	{1, 18, "\"@(#) SCO UNIX 3.2", 0, PER_SCOSVR3},
	{1, 17, "@(#) SCO UNIX 3.2", 0, PER_SCOSVR3},
	{1, 11, "@(#)SCO 3.2", 0, PER_SCOSVR3},

	/*
	 * SCO Unix 3.2.4.2 OpenServer 5 gives 32 bit inodes except
	 * programs compiled with ods30 compatibilty. In fact OS5
	 * always gives 32 bits but the library drops the top 16 in
	 * odt30 mode. We know what should happen and do it however.
	 */
	{0, 32, "@(#) crt1.s.source 20.1 94/12/04", 0, PER_SCOSVR3},
	{1, 13, "ods_30_compat", ~0, SHORT_INODE},

	/*
	 * Interactive (ISC) 4.0.
	 */
	{1, -1, "INTERACTIVE", 0, PER_ISCR4},

	/*
	 * End of table.
	 */
	{0, 0, 0, 0, 0}
};

/*
 * Parse a comments section looking for clues as to the system this
 * was compiled on so we can get the system call interface right.
 */
static int
coff_parse_comments(struct file *fp, COFF_SCNHDR *sect, long *personality)
{
	u_long			offset, nbytes;
	int			hits = 0, err;
	char			*buffer;

	if (!(buffer = (char *)__get_free_page(GFP_KERNEL)))
		return 0;

	/*
	 * Fetch the size of the section.  There must be something in there
	 * or the section wouldn't exist at all.  We only bother with the
	 * first 8192 characters though.  There isn't any point getting too
	 * carried away!
	 */
	if ((nbytes = COFF_LONG(sect->s_size)) > 8192)
		nbytes = 8192;

	offset = COFF_LONG(sect->s_scnptr);
	while (nbytes > 0) {
		u_long		count, start = 0;
		char		*p;

		err = kernel_read(fp, offset, buffer,
				nbytes > PAGE_SIZE ? PAGE_SIZE : nbytes);

		if (err <= 0) {
			 free_page((u_long) buffer);
			 return 0;
		}

		p = buffer;
		for (count = 0; count < err; count++) {
			coff_clue		*clue;
			char			c;
			
			c = *(buffer + PAGE_SIZE - 1);
			*(buffer + PAGE_SIZE - 1) = '\0';

			for (clue = coff_clues; clue->len; clue++) {
				if ((clue->len < 0 && strstr(p, clue->text)) ||
				    (clue->len > 0 && !strncmp(p, clue->text, clue->len))) {
					*personality &= clue->mask_and;
					*personality |= clue->mask_or;
					if (clue->terminal) {
						free_page((u_long)buffer);
						return 1;
					}
					hits++;
				}
			}
			*(buffer + PAGE_SIZE - 1) = c;

			while (*p && count < err)
				p++, count++;
			if (count < err) {
				p++;
				count++;
				start = count;
			}
		}

		/*
		 * If we didn't find an end ofstring at all this page
		 * probably isn't useful string data.
		 */
		if (start == 0)
			start = err;

		nbytes -= start;
		offset += start;
	}

	free_page((u_long)buffer);
	return (hits);
}

/*
 * Small procedure to test for the proper file alignment.
 * Return the error code if the section is not properly aligned.
 */
static __inline__ int
coff_isaligned(COFF_SCNHDR *sectp)
{
	long			scnptr = COFF_LONG(sectp->s_scnptr);
	long			vaddr = COFF_LONG(sectp->s_vaddr);

	if ((vaddr - scnptr) & ~PAGE_MASK)
		return -ENOEXEC;
	return 0;
}

/*
 * Clear the bytes in the last page of data.
 */
static int
coff_clear_memory(u_long addr, u_long size)
{
	int			err = 0;

	if ((size = (PAGE_SIZE - (addr & ~PAGE_MASK)) & ~PAGE_MASK) == 0)
		goto out;
	if ((err = verify_area(VERIFY_WRITE, (void *)addr, size)) < 0)
		goto out;

	while (size-- != 0) {
		__put_user(0, (char *)addr);
		addr++;
	}

out:
	return (err);
}

static inline unsigned long
map_coff(struct file *file, coff_section *sect, unsigned long prot,
	unsigned long flag, unsigned long offset)
{
	int map_addr;

	down_write(&current->mm->mmap_sem);
	map_addr = do_mmap(file,
			   sect->vaddr & PAGE_MASK,
			   sect->size + (sect->vaddr & ~PAGE_MASK),
			   prot, flag, offset);
	up_write(&current->mm->mmap_sem);

	return (map_addr);
}

static u_long *
coff_mktables(char *p, int argc, int envc)
{
	u_long	*argv, *envp;
	u_long	*sp;

	sp = (u_long *) ((-(u_long)sizeof(char *)) & (u_long)p);

	sp -= envc + 1;
	envp = sp;
	sp -= argc + 1;
	argv = sp;
	
	put_user(envp, --sp);
	put_user(argv, --sp);
	put_user(argc, --sp);

	current->mm->arg_start = (u_long)p;

	while (argc-- > 0) {
		__put_user(p, argv++);
		p += strlen_user(p);
	}
	
	__put_user(NULL, argv);
	current->mm->arg_end = current->mm->env_start = (u_long)p;
	
	while (envc-- > 0) {
		__put_user(p, envp++);
		p += strlen_user(p);
	}

	__put_user(NULL, envp);
	current->mm->env_end = (u_long) p;

	return (sp - 1); 
}

#ifdef CONFIG_BINFMT_COFF_INFLATE
static int
uncompress(z_stream *zstream,
	  char *compressed_buffer,
	  size_t compressed_buffer_size,
	  char *uncompressed_buffer,
	  size_t *uncompressed_buffer_size)
{
	int status;

	status = zlib_inflateInit(zstream);
	if(status < 0)
		return -1;

	zstream->next_in = compressed_buffer;
	zstream->avail_in = compressed_buffer_size;
	zstream->next_out = uncompressed_buffer;
	zstream->avail_out = *uncompressed_buffer_size;

	status = zlib_inflate(zstream, Z_NO_FLUSH);
	if(status < 0)
		return -1;

	status = zlib_inflateEnd(zstream);
	if(status < 0)
		return -1;

	return 0;
}

static int
uncompress_section(z_stream *zstream,
		   struct linux_binprm *bprm,
		   loff_t compressed_buffer_position,
		   size_t compressed_buffer_size,
		   char *uncompressed_buffer,
		   size_t *uncompressed_buffer_size)
{
	char *compressed_buffer;
	char size[4];
	int status;

	if(compressed_buffer_size == 0)
		return 0;

	compressed_buffer = kmalloc(compressed_buffer_size, GFP_KERNEL);
	if(compressed_buffer == NULL) {
		printk("Cannot allocate %lu bytes\n",
		       compressed_buffer_size);
		return -ENOMEM;
	}

	if((status = COFF_READ(bprm->file, size, 4,
			       &compressed_buffer_position)) < 0)
		goto out;

	compressed_buffer_size -= 4;

	*uncompressed_buffer_size = COFF_LONG(size);

	if((status = COFF_READ(bprm->file, compressed_buffer,
			       compressed_buffer_size,
			       &compressed_buffer_position)) < 0)
		goto out;

	if(uncompress(zstream,
		      compressed_buffer, compressed_buffer_size,
		      uncompressed_buffer, uncompressed_buffer_size) < 0)
		goto out;

out:
	kfree(compressed_buffer);

	return status;
}

static int
uncompress_buffer(z_stream *zstream,
		   struct linux_binprm *bprm,
		   loff_t compressed_buffer_position,
		   size_t compressed_buffer_size,
		   char **uncompressed_buffer,
		   size_t *uncompressed_buffer_size)
{
	char *compressed_buffer;
	char *buffer;
	size_t buffer_size;
	char size[4];
	int status;

	if(compressed_buffer_size == 0)
		return 0;

	compressed_buffer = kmalloc(compressed_buffer_size, GFP_KERNEL);
	if(compressed_buffer == NULL) {
		printk("Cannot allocate %lu bytes\n",
		       compressed_buffer_size);
		return -ENOMEM;
	}

	if((status = COFF_READ(bprm->file, size, 4,
			       &compressed_buffer_position)) < 0)
		goto out;

	compressed_buffer_size -= 4;

	if((status = COFF_READ(bprm->file, compressed_buffer,
			       compressed_buffer_size,
			       &compressed_buffer_position)) < 0)
		goto out;

	buffer_size = COFF_LONG(size);
	buffer = kmalloc(buffer_size, GFP_KERNEL);
	if(buffer == NULL) { 
		status = -ENOMEM;
		printk("Cannot allocate %lu bytes\n",
		       buffer_size);
		goto out;
	}

	if(uncompress(zstream,
		      compressed_buffer, compressed_buffer_size,
		      buffer, &buffer_size) < 0) {
		kfree(*uncompressed_buffer);
		goto out;
	}

	*uncompressed_buffer = buffer;
	*uncompressed_buffer_size = buffer_size;

out:
	kfree(compressed_buffer);

	return status;
}
#endif /* CONFIG_BINFMT_COFF_INFLATE */

/*
 * Relocate a COFF section if needed.
 */
static int 
coff_relocate_binary(struct linux_binprm *bprm,
		     struct file *file,
		     char *header,
		     COFF_SCNHDR *sptr,
		     unsigned int start_addr,
		     unsigned int reloc_amount,
		     relocation_stack *relstk,
		     COFF_SYMENT *sym_arr)
{
	loff_t position;
	unsigned int n_reloc = 0;
	unsigned int instruction;
	unsigned int inst_addr;
	unsigned int label;
	int status = 0;
	COFF_RELOC *creloc_arr;
	COFF_RELOC *creloc;
	unsigned int creloc_bytes = 0;
	long reloc_size;
	unsigned short type;
	unsigned long flags;

	if (relstk->index != -1) {
		printk(KERN_ALERT "Complex relocation stack should be empty!\n");
		return -ENOEXEC;		
	}

	flags = COFF_LONG(sptr->s_flags);

#ifdef CONFIG_BINFMT_COFF_INFLATE
	if(flags & COFF_STYP_COMPR && COFF_LONG(sptr->s_nreloc)) {
		z_stream zstream;
		size_t creloc_size;
		loff_t position = COFF_LONG(sptr->s_relptr);

		zstream.workspace = kmalloc(zlib_inflate_workspacesize(),
					    GFP_KERNEL);
		if(zstream.workspace == NULL) {
			printk("BINFMT_COFF: Cannot allocate memory for uncompressing\n");
			goto out;
		}

		if((status = uncompress_buffer(&zstream, bprm,
						position,
						COFF_LONG(sptr->s_nreloc),
						&creloc_arr,
						&creloc_size)) < 0) {
			goto out;	
		}

		creloc = creloc_arr;
		reloc_size = creloc_size / COFF_RELSZ;

		kfree(zstream.workspace);
	} else {
#endif /* CONFIG_BINFMT_COFF_INFLATE */
		creloc_arr = (COFF_RELOC *) kmalloc (COFF_BUFFER_SIZE, GFP_KERNEL);
		if (creloc_arr == NULL)
			return -ENOMEM;
		reloc_size = COFF_LONG(sptr->s_nreloc);
#ifdef CONFIG_BINFMT_COFF_INFLATE
	}
#endif /* CONFIG_BINFMT_COFF_INFLATE */

	/*
	 * Read the first relocation entry, if there are any.
	 */
	for(n_reloc = 0; n_reloc < reloc_size;
			n_reloc++, creloc++, creloc_bytes -= COFF_RELSZ) {
		if(!(flags & COFF_STYP_COMPR) && creloc_bytes == 0) {
			creloc_bytes = (COFF_BUFFER_SIZE > (reloc_size - n_reloc) * COFF_RELSZ ?
					(reloc_size - n_reloc) * COFF_RELSZ :
					COFF_BUFFER_SIZE);
			position = COFF_LONG(sptr->s_relptr) + (n_reloc * COFF_RELSZ);
			if ((status = COFF_READ(file,
						creloc_arr, 
						creloc_bytes,
						&position)) < 0) {
				printk("unable to read %u bytes at %lu\n", creloc_bytes, position);
				goto out;
			}
			creloc = creloc_arr;
		}

		type = COFF_SHORT(creloc->r_type);

		if(type == R_ABS) continue;

		/*
		  * Extract the relocatable field from the object data.
		 */
		inst_addr = COFF_LONG(creloc->r_vaddr) + start_addr;

		/*------------------------------------------------------------------*/
		/* If this is a relocation expression arithmetic or push            */
		/* instruction, then we can handle it now and move on to the next   */
		/* relocation entry.                                                */
		/*------------------------------------------------------------------*/
		if (ismathrel(type)) { 
			if ((status = coff_rel_math(creloc, relstk)) < 0)
				goto out;
			continue;
		}

		if (ispushrel(type)) {
			if ( COFF_LONG(creloc->r_symndx) != -1 && type == RE_PUSH) {
				if(sym_arr == NULL) {
					COFF_SYMENT symbol;
					position = COFF_LONG(((COFF_FILHDR *)header)->f_symptr) + COFF_LONG(creloc->r_symndx) * COFF_SYMESZ;
					if ((status = COFF_READ(file, &symbol, COFF_SYMESZ, &position)) < 0)
						goto out;
					if ((status = coff_rel_push(&symbol, creloc, reloc_amount, relstk)) < 0)
						goto out;
				} else {
					COFF_SYMENT *symbol;
					position = COFF_LONG(creloc->r_symndx);
					symbol = &sym_arr[position];
					if ((status = coff_rel_push(symbol, creloc, reloc_amount, relstk)) < 0)
						goto out;
				}
			} else {
				if ((status = coff_rel_push(NULL, creloc, reloc_amount, relstk)) < 0)
					goto out;
			}
			continue;
		}

		/*-----------------------------------------------------------------*/
		/* Handle relocation expression field instructions.                */
		/*-----------------------------------------------------------------*/
		if (isstfldrel(type)) {
			if ((status = coff_rel_stfld((unsigned int *)inst_addr, creloc, relstk)) < 0)
				goto out;
			continue;
		}

		/*
		 * Modify the field based on the relocation type.
		 */
		switch (type) { 

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
		case 0x100:
		case 0x200:
			break;
			
		default:
			/*
			 * All other cases are treated as default for which future 
			 * extension are requested.
			 */
			printk(KERN_ALERT "BINFMT_COFF: type 0x%lx not supported \n",
			       COFF_SHORT(creloc->r_type));
			status = -ENOEXEC;
			goto out;
		}
	}

	if (relstk->index != -1) {
		printk(KERN_ALERT "Complex relocation stack should be empty!\n");
		status = -ENOEXEC;
		goto out;
	}

	status = 0;

out:
	kfree(creloc_arr);
	return status;
}

/*
 * Helper function to process the load operation.
 */
static int
coff_load_object(struct linux_binprm *bprm, struct pt_regs *regs, int binary)
{
	COFF_FILHDR	*coff_hdr = NULL;
	COFF_SCNHDR	*sect_bufr = NULL, *sect_ptr = NULL;
	coff_section    text = COFF_SECTION_INITIALIZER,
	                data = COFF_SECTION_INITIALIZER,
		        bss = COFF_SECTION_INITIALIZER,
		        stack = COFF_SECTION_INITIALIZER,
		        lib = COFF_SECTION_INITIALIZER,
		        global = COFF_SECTION_INITIALIZER;
	unsigned int    start_addr = 0;
	unsigned int    end_addr = 0;
	u_long          p = bprm->p;
	u_long          entry_point = 0;
	u_long          sp;
	u_long          stack_len = 0;
	short		flags, aout_size = 0;
	int		pageable = 1, sections = 0, status = 0, i;
	int		coff_exec_fileno;
	u_long          read_size, posmax = 0;
	loff_t          position;
	u_long		personality = PER_SVR3;
	mm_segment_t	old_fs;
	int             ok = 1;
	struct file    *file;
#ifdef CONFIG_BINFMT_COFF_USE_SHLIBS
	int 		coff_interpreter = 0;
	char 		interpreter_buffer[128];
#endif

	COFF_SYMENT     *sym_arr = NULL; /* symtab for compressed file */
#ifdef CONFIG_BINFMT_COFF_INFLATE
	z_stream 	zstream;
#endif /* CONFIG_BINFMT_COFF_INFLATE */
	unsigned long def_flags = 0;

        /*****************************************************************************/
        /* Complex Relocation Expression Stack                                       */
        /*****************************************************************************/
	relocation_stack relstk;
	relstk.stack = NULL;
	relstk.index = -1;
	relstk.size  = 2;

	coff_hdr    = (COFF_FILHDR *) bprm->buf;

	stack.size  = 0;
	stack.vaddr = 0;

	file = bprm->file;

#ifdef COFF_TRACE
	printk("BINFMT_COFF: file = %s, magic=0x%2.2x flags=0x%2.2x\n", 
		bprm->filename, COFF_SHORT(coff_hdr->f_magic), COFF_SHORT(coff_hdr->f_flags));
#endif

retry:
	/*
	 * Validate the magic value for the object file.
	 */
#ifdef CONFIG_TMS320C6X
	if ((COFF_C6XBADMAG(*coff_hdr)) || (COFF_C6XBADMAGTARGID(*coff_hdr)))
		return -ENOEXEC;
#else
	if (COFF_I386BADMAG(*coff_hdr))
		return -ENOEXEC;
#endif

	/*
	 * The object file should have 32 BIT little endian format.
	 */
	flags = COFF_SHORT(coff_hdr->f_flags);

#ifdef CONFIG_TMS320C6X
#ifdef CONFIG_CPU_BIG_ENDIAN
	if ((flags & COFF_F_AR32W) == 0) {
		printk(KERN_WARNING "Wrong endianness, COFF file is little endian.\n");
#else
	if ((flags & COFF_F_AR32WR) == 0) {
		printk(KERN_WARNING "Wrong endianness, COFF file is big endian.\n");
#endif
		return -ENOEXEC;
	}
#else
	if ((flags & (COFF_F_AR32WR | COFF_F_AR16WR)) != COFF_F_AR32WR)
		{ printk("bad ar32/16 flags\n"); return -ENOEXEC;}
#endif

	/*
	 * If the file is not executable then reject the execution. This means
	 * that there must not be external references.
	 */
#ifndef CONFIG_BINFMT_COFF_USE_SHLIBS
	if ((flags & COFF_F_EXEC) == 0)
		{ printk("bad exec flag\n"); return -ENOEXEC;}
#endif

#ifndef CONFIG_BINFMT_COFF_NORELOC
	/*
	 * If the file is not relocatable then reject the execution.
	 */
	if (flags & COFF_F_RELFLG) {
		printk(KERN_WARNING "COFF file is not relocatable\n");
		return -ENOEXEC;
	}
#endif

#ifndef CONFIG_BINFMT_COFF_INFLATE
	if (flags & COFF_F_COMPR) {
		printk(KERN_WARNING "Compressed COFF binaries are not supported by kernel\n");
		return -ENOEXEC;
	}
#endif /* CONFIG_BINFMT_COFF_INFLATE */
	/*
	 * Extract the header information which we need.
	 */
	sections = COFF_SHORT(coff_hdr->f_nscns);	/* Number of sections */
	aout_size = COFF_SHORT(coff_hdr->f_opthdr);	/* Size of opt. headr */

	/*
	 * There must be atleast one section.
	 */
	if (!sections) {
		printk(KERN_WARNING "No section found in COFF binary\n");
		return -ENOEXEC;
	}
	
	if (!file->f_op->mmap)
		pageable = 0;

	if (!(sect_bufr = kmalloc(sections * COFF_SCNHSZ, GFP_KERNEL))) {
		printk(KERN_WARNING "coff: kmalloc failed\n");
		return -ENOMEM;
	}
	
	status = kernel_read(file, aout_size + COFF_FILHSZ,
			(char *)sect_bufr, sections * COFF_SCNHSZ);
	if (status < 0) {
		printk(KERN_WARNING "coff: unable to read\n");
		goto out_free_buf;
	}

	sect_ptr = sect_bufr;
	for (i = 0; i < sections; i++) {
		if (stack.size == 0 && (strcmp(sect_ptr->s_name, ".stack") == 0))
			stack.size = COFF_LONG(sect_ptr->s_size);
		sect_ptr++;
	}

	/*
	 *  Looking for .interp section
	 */
	sect_ptr = sect_bufr;
	for (i = 0; i < sections; i++) {

		/*
		 * .interp section contains the full path to the COFF
		 * interpreter (ie: /lib/ld-linux.so)
		 */
		if(strcmp(sect_ptr->s_name, ".interp") == 0) {
#ifndef CONFIG_BINFMT_COFF_USE_SHLIBS
			printk(KERN_WARNING "COFF shared libraries are not supported\n");
			return -ENOEXEC;
#else
			char *interpreter_path;
			unsigned long sect_pos, sect_size;

			coff_interpreter = 1;

			sect_size = COFF_LONG(sect_ptr->s_size);
			sect_pos = COFF_LONG(sect_ptr->s_scnptr);

			interpreter_path = kmalloc(sect_size + 1, GFP_KERNEL);
			status = kernel_read(file, sect_pos, interpreter_path, sect_size + 1);
			if(status < 0) {
				printk(KERN_WARNING "coff: unable to read\n");
				goto out_free_buf;
			}
			interpreter_path[sect_size] = '\0';

			file = open_exec(interpreter_path);
			status = PTR_ERR(file);
			if(IS_ERR(file)) {
				printk(KERN_WARNING "coff: unable to open interpreter %s\n", interpreter_path);
				goto out_free_buf;
			}
			status = kernel_read(file, 0, interpreter_buffer, 128);
			if(status < 0) {
				printk(KERN_WARNING "coff: unable to read\n");
				goto out_free_buf;
			}

			coff_hdr = (COFF_FILHDR *)interpreter_buffer;
			goto retry;
#endif
		}
		sect_ptr++;
	}

	status = get_unused_fd();
	if (status < 0) {
		printk(KERN_WARNING "coff: unable to get free fs\n");
		goto out_free_buf;
	}

	get_file(file);
	fd_install(coff_exec_fileno = status, file);

	if(stack.size == 0)
		stack.size  = 4 * PAGE_SIZE;  /* default value = 16kb */

	/*
	 *  Loop through the sections and find the various types
	 */
	sect_ptr = sect_bufr;
	for (i = 0; i < sections; i++) {
		long sect_flags = COFF_LONG(sect_ptr->s_flags);
		unsigned long sect_size = COFF_LONG(sect_ptr->s_size);

#ifdef CONFIG_BINFMT_COFF_INFLATE
		/*
		 * in a compressed coff binary, if the section has a _real_
		 * content, its uncompressed size is the first word of
		 * its data.
		 */
		if(sect_flags & COFF_STYP_COMPR) {
			unsigned long sect_pos;
			char s_scnptr[4];
			
			sect_pos = COFF_LONG(sect_ptr->s_scnptr);
			if(sect_pos) {
				status = kernel_read(file,
						sect_pos, s_scnptr, 4);
				if(status < 0) {
					printk(KERN_WARNING "coff: unable to read\n");
					goto out_free_buf;
				}
				sect_size = COFF_LONG(s_scnptr);
			}
		}
#endif /* CONFIG_BINFMT_COFF_INFLATE */

		if ((strcmp(sect_ptr->s_name, ".stack") == 0)) {
			stack.vaddr = COFF_LONG(sect_ptr->s_vaddr);
			posmax = position_max(posmax, stack.size + stack.vaddr);
			sect_ptr++;
			continue;
		}		

		if (sect_flags & COFF_STYP_TEXT) {
			status |= coff_isaligned(sect_ptr);
			text.size += sect_size;
			text.vaddr = COFF_LONG(sect_ptr->s_vaddr);
			posmax = position_max(posmax, text.size + text.vaddr);
			sect_ptr++;
			continue;
		}

		if ((sect_flags & COFF_STYP_DATA) &&
		    !(sect_flags & COFF_STYP_COPY) &&
		    (strcmp(sect_ptr->s_name, ".stack") != 0) &&
		    (strcmp(sect_ptr->s_name, ".far") != 0) &&
		    !(sect_flags & COFF_STYP_BSS)) {
			status |= coff_isaligned(sect_ptr);
			data.size += sect_size;
			posmax = position_max(posmax, sect_size
					      + COFF_LONG(sect_ptr->s_vaddr));

			sect_ptr++;
			continue;
		}

		if ((sect_flags & COFF_STYP_BSS)) {
			bss.size += sect_size;
			if (strcmp(sect_ptr->s_name, ".far") != 0)
				bss.vaddr = COFF_LONG(sect_ptr->s_vaddr);
			posmax = position_max(posmax, sect_size
					      + COFF_LONG(sect_ptr->s_vaddr));
			sect_ptr++;
			continue;
		}

		if (sect_flags & COFF_STYP_LIB) {
			lib.size += sect_size;
			posmax = position_max(posmax, sect_size
					      + COFF_LONG(sect_ptr->s_vaddr));
			sect_ptr++;
			continue;
		}
		sect_ptr++;
	}

	/*
	 * If any of the sections weren't properly aligned we aren't
	 * going to be able to demand page this executable. Note that
	 * at this stage the *only* excuse for having status <= 0 is if
	 * the alignment test failed.
	 */
	if (status < 0)
		pageable = 0;

	/*
	 * Ensure that there are the required sections.  There must be one or
	 * more text sections and one each of the data and bss sections for an
	 * executable. A library may or may not have a data / bss section.
	 */
	if (!text.size || (binary && !(data.size || bss.size))) {
		status = -ENOEXEC;
		printk(KERN_WARNING "coff: missing required text or data segment\n");		
		goto out_free_file;
	}

	/*
	 * If there is no additional header then assume the file starts
	 * at the first byte of the text section.  This may not be the
	 * proper place, so the best solution is to include the optional
	 * header.  A shared library __MUST__ have an optional header to
	 * indicate that it is a shared library.
	 */
	if (aout_size == 0) {
		if (!binary) {
			status = -ENOEXEC;
			printk(KERN_WARNING "coff: invalid library\n");		
			goto out_free_file;
		}
		entry_point = 0;
	} else if (aout_size < (short) COFF_AOUTSZ) {
		status = -ENOEXEC;
		printk(KERN_WARNING "coff: invalid a.out size\n");		
		goto out_free_file;
	} else {
		COFF_AOUTHDR *aout_hdr;
		short	     aout_magic;

		aout_hdr = (COFF_AOUTHDR *) &((char *)coff_hdr)[COFF_FILHSZ];
		aout_magic = COFF_SHORT(aout_hdr->magic);

		/*
		 * Validate the magic number in the a.out header. If it is valid then
		 * update the starting symbol location. Do not accept these file formats
		 * when loading a shared library.
		 */
		switch (aout_magic) {
		case COFF_OMAGIC:
		case COFF_ZMAGIC:
		case COFF_STMAGIC:
		case COFF_DMAGIC:
			if (!binary) {
				printk("bad magic for lib\n");
				status = -ENOEXEC;
				goto out_free_file;
			}
			entry_point = (u_int) COFF_LONG(aout_hdr->entry);
			break;

		/*
		 * Magic value for a shared library. This is valid only when
		 * loading a shared library.
		 *
		 * (There is no need for a start_addr. It won't be used.)
		 */
		case COFF_SHMAGIC:
			if (!binary)
				break;
		default:
			status = -ENOEXEC;
			printk(KERN_WARNING "coff: invalid a.out magic\n");		
			goto out_free_file;
		}
	}
	/*
	 *  THIS IS THE POINT OF NO RETURN. THE NEW PROCESS WILL TRAP OUT SHOULD
	 *  SOMETHING FAIL IN THE LOAD SEQUENCE FROM THIS POINT ONWARD.
	 */

#ifdef COFF_TRACE
	printk("BINFMT_COFF: headers OK, will load or fault\n");
#endif
	/*
	 * Flush the executable from memory. At this point the executable is
	 * committed to being defined or a segmentation violation will occur.
	 */
	if ((!binary) || ((status = flush_old_exec(bprm))))
		goto out_free_file;

	current->mm->start_data = 0;
	current->mm->end_data = 0;
	current->mm->end_code = 0;
	current->flags &= ~PF_FORKNOEXEC;
	current->mm->def_flags = def_flags;

	/*
	 * Look for clues as to the system this binary was compiled
	 * on in the comments section(s).
	 *
	 * Only look at the main binary, not the shared libraries
	 * (or would it be better to prefer shared libraries over
	 * binaries?  Or could they be different???)
	 */
	sect_ptr = sect_bufr;
	for (i = 0; i < sections; i++) {
		long sect_flags = COFF_LONG(sect_ptr->s_flags);
		
		if (sect_flags == COFF_STYP_INFO && 
		    (status = coff_parse_comments(file,
						  sect_ptr, &personality)) > 0)
			goto found;
		
		sect_ptr++;
	}

	/*
	 * If no .comments section was found there is no way to
	 * figure out the personality. Odds on it is SCO though...
	 */
	personality = PER_SCOSVR3;
found:
	set_personality(personality);
	
	/*
	 * We have to add the size of our arguments to our stack size
	 * otherwise it's too easy for users to create stack overflows
	 * by passing in a huge argument list.  And yes,  we have to be
	 * pedantic and include space for the argv/envp array as it may have
	 * a lot of entries.
	 */

#define TOP_OF_ARGS (PAGE_SIZE * MAX_ARG_PAGES - sizeof(void *))

	stack_len  = TOP_OF_ARGS - bprm->p;             /* the strings */
	stack_len += (bprm->argc + 1) * sizeof(char *); /* the argv array */
	stack_len += (bprm->envc + 1) * sizeof(char *); /* the envp array */
	if (stack_len > stack.size)
		stack.size =  stack_len;

	/*
	 * Compute sizes and allocate memory.
	 */
	global.size  = PAGE_ALIGN(posmax);
	global.vaddr = 0;
	
	status = map_coff(file,
			  &global,
			  PROT_READ | PROT_WRITE | PROT_EXEC,
			  MAP_PRIVATE,
			  0);
	
	if ((status < 0) && (status > -4096)) {
		printk("coff: do_mmap failed, aborting.\n");
		goto sigsegv;
	}

	start_addr = (unsigned int) status;
	end_addr   = PAGE_ALIGN(start_addr + global.size);
	posmax     = start_addr;

	/*
	 * Clean up memory
	 */
	memset((void *) start_addr, 0, (size_t) global.size);

#ifdef COFF_TRACE
	printk("\n");
	printk("BINFMT_COFF: file = %s\n", bprm->filename);
	printk("BINFMT_COFF: memory allocation at 0x%x\n", start_addr);
	printk("BINFMT_COFF: total size : %d\n", global.size);
#ifdef COFF_TRACE_VERBOUS
       	show_free_areas();
#endif
#endif
	
	/*
	 * Align starting code address to fetch packet
	 */
	start_addr   = ALIGN_FETCH_PTR(start_addr); 

#ifdef COFF_TRACE
	printk("BINFMT_COFF: alignment for fetch packet to %08lx\n", start_addr);
#endif
	/*
	 * Create new memory context.
	 */
	current->mm->end_code = text.size + text.vaddr +
		(current->mm->start_code = start_addr);
	current->mm->end_data = data.size +
		(current->mm->start_data = current->mm->end_code);
	current->mm->brk = bss.size +
		(current->mm->start_brk = current->mm->end_data);

	install_exec_creds(bprm);
	
	entry_point += start_addr;
	regs->dp = start_addr + bss.vaddr;

#ifdef COFF_TRACE
	printk ("BINFMT_COFF: entry_point = %08lx\n", entry_point);
	printk ("BINFMT_COFF: data base pointer (dp) = %08lx\n", regs->dp);
#endif

	relstk.stack = (unsigned int *) kmalloc (relstk.size * sizeof(unsigned int), GFP_KERNEL);
	if (relstk.stack == NULL) {
		status = -ENOMEM;
		goto sigsegv;
	}

#ifdef CONFIG_BINFMT_COFF_INFLATE
	if(flags & COFF_F_COMPR) {
		unsigned long compressed_size, uncompressed_size;
		loff_t position;
		int err;

		zstream.workspace = kmalloc(zlib_inflate_workspacesize(), GFP_KERNEL);	

		position = COFF_LONG(((COFF_FILHDR *)bprm->buf)->f_symptr);
		if(position > 0) {
			compressed_size = COFF_LONG(((COFF_FILHDR *)bprm->buf)->f_nsyms);
			if((status = uncompress_buffer(&zstream, bprm,
						       position,
						       compressed_size,
						       &sym_arr,
						       &uncompressed_size)) < 0)
				goto sigsegv;
		}
	}
#endif /* CONFIG_BINFMT_COFF_INFLATE */

	/*
	 * Load all sections one at a time.
	 */
	sect_ptr = sect_bufr;
	for (i = 0; i < sections && ok ; i++) {
		/*
		 * Ignore empty sections or sections whose flags indicate the
		 * section is not to be loaded.  if the CLEAR_BSS flag is set, 
		 * BSS is "loaded" even though it has no data.  the cinit 
		 * section is deferred until later to ensure bss is loaded 
		 * first.
		 */
		unsigned long sect_flags = COFF_LONG(sect_ptr->s_flags);

		if ((COFF_LONG(sect_ptr->s_scnptr) || IS_BSS(sect_ptr)) &&
		    COFF_LONG(sect_ptr->s_size) &&   
		    !(sect_flags & COFF_STYP_DSECT) &&
		    !(sect_flags & COFF_STYP_COPY) &&
		    !IS_CINIT(sect_ptr) && 
		    !(sect_flags & COFF_STYP_NOLOAD))
			{           
				unsigned long buf = COFF_LONG(sect_ptr->s_vaddr) + start_addr;
				position = COFF_LONG(sect_ptr->s_scnptr);
#ifdef COFF_TRACE
				printk("BINFMT_COFF: loading: pos=0x%08lx, vaddr=0x%08lx,"
				       "size=0x%lx, align=0x%lx, nreloc=0x%lx\n",
				       buf, COFF_LONG(sect_ptr->s_vaddr) ,
				       COFF_LONG(sect_ptr->s_size),
				       COFF_ALIGN_SIZE(sect_flags),
				       COFF_LONG(sect_ptr->s_nreloc)
				       );
#endif
				ok &= 1;                      
#ifdef CONFIG_BINFMT_COFF_INFLATE
				if(sect_flags & COFF_STYP_COMPR) {
					size_t uncompressed_size;

					if((status = uncompress_section(
							&zstream, bprm,
							position,
							COFF_LONG(sect_ptr->s_size),
							buf,
							&uncompressed_size)) < 0) {
						goto sigsegv;
					}

					read_size = uncompressed_size;
				} else 
#endif /* CONFIG_BINFMT_COFF_INFLATE */
					read_size = COFF_READ(file,
							      buf,
							      COFF_LONG(sect_ptr->s_size),
							      &position);
#ifdef COFF_TRACE
				printk("BINFMT_COFF: read 0x%lx \n",read_size);
				printk("BINFMT_COFF: move to: 0x%08lx (size: 0x%08lx)\n",
				       buf,
				       COFF_LONG(sect_ptr->s_size));
#endif
				posmax = position_max(posmax,
						      buf + read_size);
#ifdef COFF_TRACE
				printk("\n");
#endif
				/*
				 * Eventually relocate the loaded section.
				 */
				if ((status = coff_relocate_binary(bprm, file, coff_hdr, sect_ptr, start_addr, start_addr, &relstk, sym_arr)) < 0) {
					printk("error while relocating\n");
					goto sigsegv;
				}
			}
		sect_ptr++;
	}

	/*
	 * Load CINIT sections.
	 */
	sect_ptr = sect_bufr;
	for (i = 0; i < sections && ok; i++) {  
		unsigned int *copy_bufr = NULL;
		int read_size;
		
		if (IS_CINIT(sect_ptr)) {
			int length;
			const unsigned int *recptr;
#ifdef COFF_TRACE
			printk("BINFMT_COFF: CINIT section found\n");
#endif
#ifdef CONFIG_BINFMT_COFF_INFLATE
			if(COFF_LONG(sect_ptr->s_flags) & COFF_STYP_COMPR) {
				unsigned long compressed_size = COFF_LONG(sect_ptr->s_size);
				unsigned long uncompressed_size;

				position = COFF_LONG(sect_ptr->s_scnptr);

				if((status = uncompress_buffer(
						&zstream, bprm,
						position,
						compressed_size,
						&copy_bufr,
						&uncompressed_size)) < 0)
					goto sigsegv;
			} else
#endif /* CONFIG_BINFMT_COFF_INFLATE */
			{
				copy_bufr = (unsigned int *) kmalloc(COFF_LONG(sect_ptr->s_size) + 8, 
						GFP_KERNEL);
				if (copy_bufr == NULL) {
					status = -ENOMEM;
					goto sigsegv;
				}

				position = COFF_LONG(sect_ptr->s_scnptr);
				read_size = COFF_READ(file,
						copy_bufr,
						COFF_LONG(sect_ptr->s_size),
						&position);
#ifdef COFF_TRACE
				printk(KERN_ALERT "BINFMT_COFF: copy to  0x%08lx (size:0x%08lx)\n",
						copy_bufr,COFF_LONG(sect_ptr->s_size));
#endif

				if (read_size != COFF_LONG(sect_ptr->s_size)) {
					status = -ENOEXEC;
					goto sigsegv;
				}
			}
			/*
			 * Eventually relocate the loaded section.
			 */
			if ((status = coff_relocate_binary(bprm,
							   file, coff_hdr,
							   sect_ptr,
							   (unsigned long) copy_bufr 
							   - COFF_LONG(sect_ptr->s_vaddr),
							   start_addr,
							   &relstk,
							   sym_arr)) < 0)
				goto sigsegv;

			recptr = copy_bufr;
            		while((length = *recptr++) != 0) {
				if (length < 0){
					printk("warning: negative length\n");    
					recptr = ALIGN_COFF_PTR(recptr);
				} else {
					char *to    = (char *)*recptr++;
					char *from  = (char *)recptr;
					
					posmax = position_max(posmax, (long) to + length);

#ifdef COFF_TRACE
					printk("BINFMT_COFF: initializing memory block at 0x%x from 0x%x (size: %d)\n",
					       to, from, length);
#endif                      
					memcpy(to, from, length);
					
					from += length;
					recptr = ALIGN_COFF_PTR(from);
				}
			}
#ifdef COFF_TRACE
			printk("\n");
#endif
		}
		if (copy_bufr)
			kfree(copy_bufr);
		
		sect_ptr++;
	}
	
#if defined(CONFIG_BINFMT_COFF_INFLATE)
	if(flags & COFF_F_COMPR)
		kfree(zstream.workspace);
#endif /* CONFIG_BINFMT_COFF_INFLATE */

	flush_icache_range(current->mm->start_code, current->mm->end_code);

	/* Clear extra memory */
#ifdef COFF_TRACE
	printk("BINFMT_COFF: cleaning extra memory from 0x%x, size 0x%x\n",
	       posmax, (end_addr - posmax));
#endif
	if (posmax > end_addr)
		printk(KERN_WARNING "coff: data beyond allocated space! %p %p\n",
		       posmax, end_addr);
	else
		memset((void *)posmax, 0, (end_addr - posmax));	

	/* Calculate stack pointer */
	sp = stack.vaddr ? start_addr + stack.vaddr + stack.size : end_addr;
	current->mm->start_stack = sp;
	
	/* Copy the arg pages onto the stack */
	for (i = TOP_OF_ARGS - 1; i >= bprm->p; i--)
		* (char *) --sp =
			((char *) page_address(bprm->page[i/PAGE_SIZE]))[i % PAGE_SIZE];
	
	/* Construct the parameter and environment string table entries */
	sp = (u_long) coff_mktables((char *) sp, bprm->argc, bprm->envc);
	
#ifdef COFF_TRACE
	printk("BINFMT_COFF: start_stack : 0x%x\n" , current->mm->start_stack);
	printk("BINFMT_COFF: brk         : 0x%x\n" , current->mm->brk);
	printk("BINFMT_COFF: start_brk   : 0x%x\n" , current->mm->start_brk);
	printk("BINFMT_COFF: end_data    : 0x%x\n" , current->mm->end_data);
	printk("BINFMT_COFF: end_code    : 0x%x\n" , current->mm->end_code);
	printk("BINFMT_COFF: start_code  : 0x%x\n" , current->mm->start_code);
	printk("BINFMT_COFF: bprm->argc  : %d, bprm->envc: %d\n", bprm->argc, bprm->envc);
	printk("BINFMT_COFF: entry point : 0x%x\n", entry_point);
#endif

	start_thread(regs, entry_point, sp);

	set_binfmt(&coff_format);
	
	/*
	 * Generate any needed trap for this process. If an error occured then
	 * generate a segmentation violation. If the process is being debugged
	 * then generate the load trap. (Note: If this is a library load then
	 * do not generate the trap here. Pass the error to the caller who
	 * will do it for the process in the outer lay of this procedure call.)
	 */
	if (status < 0) {
sigsegv:
		printk(KERN_WARNING "coff: trapping process with SEGV\n");
		send_sig(SIGSEGV, current, 0);	/* Generate the error trap  */
	} else if (current->ptrace & PT_PTRACED)
		send_sig(SIGTRAP, current, 0);

	/* We are committed. It can't fail */
	status = 0;

out_free_file:
	sys_close(coff_exec_fileno);

out_free_buf:
	if(sym_arr)
		kfree(sym_arr);
	kfree(sect_bufr);
	kfree(relstk.stack);
	return (status);
}

/*
 * Load the image for an (coff) binary.
 * 
 *   => this procedure is called by the main load sequence,
 *      it will load the executable and prepare it for execution
 */
static int
coff_load_binary(struct linux_binprm *bpp, struct pt_regs *rp)
{
	return (coff_load_object(bpp, rp, 1));
}

static int __init init_coff_binfmt(void)
{
	return register_binfmt(&coff_format);
}

static void __exit exit_coff_binfmt(void)
{
	unregister_binfmt(&coff_format);
}

core_initcall(init_coff_binfmt);
module_exit(exit_coff_binfmt);
MODULE_LICENSE("GPL");
