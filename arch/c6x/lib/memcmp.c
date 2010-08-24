/*
 *  linux/arch/c6x/lib/memcmp.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/types.h>

int memcmp(const void * cs,const void * ct,size_t count)
{
	const unsigned char *su1, *su2;

	for( su1 = cs, su2 = ct; 0 < count; ++su1, ++su2, count--)
		if (*su1 != *su2)
			return((*su1 < *su2) ? -1 : +1);
	return(0);
}

