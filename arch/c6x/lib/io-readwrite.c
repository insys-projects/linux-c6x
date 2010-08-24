/*
 *  linux/arch-c6x/io-readwrite.c
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2007, 2009 Texas Instruments Incorporated
 *  Author: Nicolas Videau (nicolas.videau@virtuallogix.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/types.h>

/* FIXME: no non-alignment support */

void __raw_writesb(void __iomem *addr, const void *data, int bytelen)
{
	volatile unsigned char *tmp1 = addr, *tmp2 = data;
	int cpt;

	for (cpt=0; cpt<bytelen; cpt++)
		*tmp1 = *tmp2++;

	return;
}

void __raw_writesw(void __iomem *addr, const void *data, int wordlen)
{
	volatile unsigned short *tmp1 = addr, *tmp2 = data;
	int cpt;

	for (cpt=0; cpt<wordlen; cpt++)
		*tmp1 = *tmp2++;

	return;
}

void __raw_writesl(void __iomem *addr, const void *data, int longlen)
{
	volatile unsigned long *tmp1 = addr, *tmp2 = data;
	int cpt;

	for (cpt=0; cpt<longlen; cpt++)
		*tmp1 = *tmp2++;

	return;
}

void __raw_readsb(void __iomem *addr, void *data, int bytelen)
{
	volatile unsigned char *tmp1 = addr, *tmp2 = data;
	int cpt;

	for (cpt=0; cpt<bytelen; cpt++)
		*tmp2++ = *tmp1;

	return;
}

void __raw_readsw(void __iomem *addr, void *data, int wordlen)
{
	volatile unsigned short *tmp1 = addr, *tmp2 = data;
	int cpt;

	for (cpt=0; cpt<wordlen; cpt++)
		*tmp2++ = *tmp1;

	return;
}

void __raw_readsl(void __iomem *addr, void *data, int longlen)
{
	volatile unsigned long *tmp1 = addr, *tmp2 = data;
	int cpt;

	for (cpt=0; cpt<longlen; cpt++)
		*tmp2++ = *tmp1;

	return;
}
