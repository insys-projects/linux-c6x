/*
 * arch/c6x/boot/compressed/misc.c
 *
 * This is a collection of several routines from gzip-1.0.3
 * adapted for Linux.
 *
 * malloc by Hannu Savolainen 1993 and Matthias Urlichs 1994
 *
 * Adapted for C6x by Aurelien Jacquiot <a-jacquiot@ti.com>
 * Copyright (C) 2012 Texas Instruments Incorporated
 *
 */
#include <asm/uaccess.h>
#include <asm/page.h>

/*
 * gzip declarations
 */
#define STATIC static

#undef memset
#undef memcpy
#define memzero(s, n)     memset ((s), 0, (n))

/* cache.c */
#define CACHE_ENABLE      0
#define CACHE_DISABLE     1
extern void cache_control(unsigned int command);

extern char input_data[];
extern int input_len;
static unsigned char *output;

static void error(char *m);

int puts(const char *);

extern int _text;		/* Defined in vmlinux.lds.S */
extern int _end;
extern int _bss_start;
extern int _bss_end;

static unsigned long free_mem_ptr;
static unsigned long free_mem_end_ptr;

#ifdef CONFIG_HAVE_KERNEL_BZIP2
#define HEAP_SIZE	0x400000
#else
#define HEAP_SIZE	0x10000
#endif

#ifdef CONFIG_KERNEL_GZIP
#include "../../../../lib/decompress_inflate.c"
#endif

#ifdef CONFIG_KERNEL_BZIP2
#include "../../../../lib/decompress_bunzip2.c"
#endif

#ifdef CONFIG_KERNEL_LZMA
#include "../../../../lib/decompress_unlzma.c"
#endif

#ifdef CONFIG_KERNEL_LZO
#include "../../../../lib/decompress_unlzo.c"
#endif

int puts(const char *s)
{
	return 0;
}

void* memset(void* s, int c, size_t n)
{
	int i;
	char *ss = (char*)s;

	for (i=0;i<n;i++) ss[i] = c;
	return s;
}

void* memcpy(void* __dest, __const void* __src,
			    size_t __n)
{
	int i;
	char *d = (char *)__dest, *s = (char *)__src;

	for (i=0;i<__n;i++) d[i] = s[i];
	return __dest;
}

static void error(char *x)
{
	puts("\n\n");
	puts(x);
	puts("\n\n -- System halted");

	while(1);	/* Halt */
}

#define STACK_SIZE (0x8000)
long __attribute__ ((aligned(8))) user_stack[STACK_SIZE];
long *stack_start = &user_stack[STACK_SIZE];

unsigned long decompress_kernel(void)
{
	free_mem_ptr     = (unsigned long)&_end;
	free_mem_end_ptr = free_mem_ptr + HEAP_SIZE;
	
	/* Start of physical kernel */
	output = (unsigned char *) CONFIG_PAGE_OFFSET;

	puts("Uncompressing Linux... ");

	/* Enable cache */
	cache_control(CACHE_ENABLE);

	decompress(input_data, input_len, NULL, NULL, output, NULL, error);

	/* Disable cache */
	cache_control(CACHE_DISABLE);

	puts("Ok, booting the kernel.\n");

	return (unsigned long) output;
}
