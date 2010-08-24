/*
 * tags.c
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *
 *  FIXME: Dual BSD/GPL License Text
 */

#include "tags.h"

#define TAG_NEXT(p) \
	((struct tag_header *)((char *)p + (((struct tag_header *)p)->size) + sizeof(struct tag_header)))

struct tag_header *c6x_tag_next(struct tag_header *atag)
{
	if (atag->tag == TAG_EOL)
		return (struct tag_header *)0;

	atag = TAG_NEXT(atag);
	if (atag->tag == TAG_EOL)
		return (struct tag_header *)0;

	return atag;
}

void *c6x_tag_find(struct tag_header *atag, unsigned int tag)
{
	while(atag && atag->tag != TAG_EOL && atag->tag != tag)
		atag = c6x_tag_next(atag);
	if (atag && atag->tag == tag)
		return (void *)((unsigned long)atag + sizeof(*atag));
	return (void *)0;
}

int c6x_tags_are_valid(struct tag_header *atag)
{
	struct tag_header *next;

	if (atag && atag->tag != TAG_SOL || atag->size)
		return 0;
	return 1;
}


