/*
 *  linux/arch/c6x/lib/coff.c
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
#include <linux/coff.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/bug.h>

#if 0
#define DEBUGP printk
#else
#define DEBUGP(fmt...)
#endif

/*****************************************************************************/
/* coff_rel_push - Push a value from a leaf relocation expression instruction*/
/*        onto the complex relocation expression stack.                      */
/*****************************************************************************/
int coff_rel_push(COFF_SYMENT *sym,
		  COFF_RELOC *creloc,
		  unsigned int reloc_amount,
		  relocation_stack *relstk)
{
	unsigned int    *val = NULL;
	int status;

	/*------------------------------------------------------------------------*/
	/* Push a new entry onto the top of the stack, growing the stack, if      */
	/* necessary.  Clear out the value before we load anything into it.       */
	/*------------------------------------------------------------------------*/
	if ((relstk->index + 1) >= relstk->size)
	{
		unsigned int *old_relstk = relstk->stack;

		relstk->size *= 2;
		relstk->stack = (unsigned int *) kmalloc (relstk->size * sizeof(unsigned int), GFP_KERNEL);
		if (relstk->stack == NULL)
			return -ENOMEM;

		memcpy(relstk->stack, old_relstk, relstk->size/2);
		kfree(old_relstk);
	}

	val = relstk->stack + (++relstk->index);

	/*------------------------------------------------------------------------*/
	/* Resolve the value we wish to push.                                     */
	/*------------------------------------------------------------------------*/
	switch (COFF_SHORT(creloc->r_type))
	{
	/*---------------------------------------------------------------------*/
	/* Just convert constants into extended precision values.              */
	/*---------------------------------------------------------------------*/
	case RE_PUSHUK:
		*val = COFF_LONG(creloc->r_symndx);
		break;
				   
        /*---------------------------------------------------------------------*/
        /* For external symbol references, read the slot vector to get the     */
        /* symbol address.                                                     */
        /* If the symbol is defined in this section, use the address of the    */
        /* current section.  The address of the symbol should appear as a      */
        /* constant push somewhere in the expression.                          */
        /*---------------------------------------------------------------------*/
	case RE_PUSH:
	{
		/*------------------------------------------------------------------*/
		/* A sym index of -1 indicates an "internal relocation", meaning    */
		/* the reference is to the current section.  If writing the         */
		/* reloc entry back out, change it to a "PUSHPC", which is also     */
		/* a reference to the current section, but with a displacement      */
		/* equal to the position of the input section within the output     */
		/* section (because in the next link, that outsect will be the      */
		/* "current section".                                               */
		/*------------------------------------------------------------------*/
		if (sym == NULL)
			return -ENOEXEC;

		/*------------------------------------------------------------------*/
		/* In the normal case, lookup the symbol in the slot vector and     */
		/* push its value.                                                  */
		/*------------------------------------------------------------------*/

		*val = COFF_LONG(sym->e_value) + reloc_amount;
	}		
	break;

        /*--------------------------------------------------------------------------*/
        /* Whoops!!! This shouldn't have happened.  Check call to coff_rel_push().  */
        /*--------------------------------------------------------------------------*/
	default:
		printk(KERN_ALERT "illegal reloc expr push instruction : type=0x%x\n", COFF_SHORT(creloc->r_type));
		return -ENOEXEC;
	}

	return 0;
}

/*****************************************************************************/
/* coff_rel_math - Perform an arithmetic operation on the value(s) at the top*/
/*        of the complex relocation expression stack.  Consume values needed */
/*          to do operation, and place result on the top of the stack.       */
/*****************************************************************************/
int coff_rel_math(COFF_RELOC *creloc, relocation_stack *relstk)
{
	unsigned int *lhs = NULL;
	unsigned int *rhs = NULL;

	/*------------------------------------------------------------------------*/
	/* Set up pointer to unary operator operand.                              */
	/*------------------------------------------------------------------------*/
	if (isunary(COFF_SHORT(creloc->r_type))) {
		if (relstk->index < 0) {
			printk(KERN_ALERT "No value on relocation stack!\n");
			return -ENOEXEC;
		}
	}

	/*------------------------------------------------------------------------*/
	/* Set up pointers to left and right side of binary operator.             */
	/*------------------------------------------------------------------------*/
	if (isbinary(COFF_SHORT(creloc->r_type))) {
		if (relstk->index < 1) {
			printk(KERN_ALERT "Lack values on relocation stack!\n");
			return -ENOEXEC;
		}
		rhs = relstk->stack + (relstk->index--);
	}

	lhs = relstk->stack + relstk->index;

	/*------------------------------------------------------------------------*/
	/* Perform operation.                                                     */
	/*------------------------------------------------------------------------*/
	switch (COFF_SHORT(creloc->r_type)) {
	case RE_ADD:  
		*lhs = *lhs + *rhs;
		break;
	case RE_SUB:  
		*lhs = *lhs - *rhs;
		break;
	case RE_DIV:  
		*lhs = *lhs / *rhs;
		break;

	case RE_SR:
		*lhs = *lhs >> *rhs;
		break;
	
	case RE_AND:  
		*lhs = *lhs & *rhs;
		break;

	default:
		printk(KERN_ALERT "unrecognized reloc expr operator! : op=0x%x\n", COFF_SHORT(creloc->r_type));
		return -ENOEXEC;
	}

	return 0;
}

/*****************************************************************************/
/* coff_rel_stfld - Store the value at the top of the complex relocation     */
/*        expression stack into the relocation field specified in the STFLD  */
/*           relocation instruction.  This function will consume one item    */
/*           from the complex relocation expression stack, leaving the stack */
/*           empty.                                                          */
/*****************************************************************************/
int coff_rel_stfld(unsigned int *addr, COFF_RELOC *creloc, relocation_stack *relstk)
{
	unsigned int *val;

	if (relstk->index < 0) {
		printk(KERN_ALERT "No value on relocation stack!\n");
		return -ENOEXEC;
	}

	val = relstk->stack + (relstk->index--);

	switch (COFF_SHORT(creloc->r_type)) {
	case RE_USTFLD:
	case RE_SSTFLD:
		*addr &= 0xFF80007F;
		*addr |= *val << 7;
		break;
	case RE_XSTFLD:
		*addr = *val;
		break;

	default:
		printk(KERN_ALERT "unrecognized reloc expr store instruction! : type=0x%x\n", COFF_SHORT(creloc->r_type));
		return -ENOEXEC;
	}

	return 0;
}

