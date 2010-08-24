/*
 * tags.h
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Mark Salter <msalter@redhat.com>
 *
 *  FIXME: Dual BSD/GPL License Text
 */

#define TAGS_MAGIC	0x54694265

/* The list must start with an TAG_SOL */
#define TAG_SOL		0x64000001

/* The list must end with an TAG_EOL */
#define TAG_EOL		0x64000002

#define TAG_CMDLINE	0x64000003

#ifndef __ASSEMBLY__
struct tag_header {
	unsigned int size;
	unsigned int tag;
};

struct tag_cmdline {
	char	cmdline[0];
};

extern int c6x_tags_are_valid(struct tag_header *atag);
extern struct tag_header *c6x_tag_next(struct tag_header *atag);
extern void *c6x_tag_find(struct tag_header *atag, unsigned int tag);
#endif
