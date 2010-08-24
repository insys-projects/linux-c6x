/*
 *  linux/include/asm-c6x/io.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot (aurelien.jacquiot@jaluna.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#ifndef __ASM_C6X_IO_H
#define __ASM_C6X_IO_H

#ifdef __KERNEL__

#include <linux/types.h>
#include <asm/virtconvert.h>

#define IO_ADDRESS(x) (x)
#define __REG(x)      (*((volatile unsigned int *)(x)))

static inline unsigned char __readb(volatile unsigned char *addr) { return (*addr); }
static inline unsigned short __readw(volatile unsigned short *addr) { return (*addr); }
static inline unsigned long __readl(volatile unsigned int *addr) { return (*addr); }

#define readb(addr) __readb((volatile unsigned char *) (addr))
#define readw(addr) __readw((volatile unsigned short *) (addr))
#define readl(addr) __readl((volatile unsigned int *) (addr))

#define writeb(b,addr) ((*(volatile unsigned char *) (addr)) = (b))
#define writew(b,addr) ((*(volatile unsigned short *) (addr)) = (b))
#define writel(b,addr) ((*(volatile unsigned int *) (addr)) = (b))

static inline unsigned char get_user_byte_io(volatile const char * addr)
{
	register unsigned char _v;

	_v = *addr;
	return _v;
}

#define inb_p(addr) get_user_byte_io((volatile char *)(addr))
#define inb(addr)   get_user_byte_io((volatile char *)(addr))

static inline void put_user_byte_io(char val,volatile char *addr)
{
	*addr = val;
}
#define outb_p(x,addr) put_user_byte_io((x),(volatile char *)(addr))
#define outb(x,addr)   put_user_byte_io((x),(volatile char *)(addr))

#define in_le16(addr) \
    ({ unsigned short __v = le16_to_cpu(*(volatile unsigned short *) (addr)); __v; })
#define in_le32(addr) \
    ({ unsigned int __v = le32_to_cpu(*(volatile unsigned int *) (addr)); __v; })
#define out_le16(addr,w) (void)((*(volatile unsigned short *) (addr)) = cpu_to_le16(w))
#define out_le32(addr,l) (void)((*(volatile unsigned int *) (addr)) = cpu_to_le32(l))

#define inw(port)        in_le16(port)
#define inl(port)        in_le32(port)

#define outw(val,port)   out_le16((port),(val))
#define outl(val,port)   out_le32((port),(val))

#define IO_SPACE_LIMIT 0xffffffff

/* Values for nocacheflag and cmode */
#define IOMAP_FULL_CACHING		0
#define IOMAP_NOCACHE_SER		1
#define IOMAP_NOCACHE_NONSER		2
#define IOMAP_WRITETHROUGH		3

extern void *__ioremap(unsigned long physaddr, unsigned long size, int cacheflag);
extern void __iounmap(void *addr, unsigned long size);

static inline void *ioremap(unsigned long physaddr, unsigned long size)
{
	return __ioremap(physaddr, size, IOMAP_NOCACHE_SER);
}
static inline void *ioremap_nocache(unsigned long physaddr, unsigned long size)
{
	return __ioremap(physaddr, size, IOMAP_NOCACHE_SER);
}
static inline void *ioremap_writethrough(unsigned long physaddr, unsigned long size)
{
	return __ioremap(physaddr, size, IOMAP_WRITETHROUGH);
}
static inline void *ioremap_fullcache(unsigned long physaddr, unsigned long size)
{
	return __ioremap(physaddr, size, IOMAP_FULL_CACHING);
}

extern void iounmap(void *addr);

/* Nothing to do */
#define dma_cache_inv(_start,_size)		do { } while (0)
#define dma_cache_wback(_start,_size)		do { } while (0)
#define dma_cache_wback_inv(_start,_size)	do { } while (0)

/*
 * Convert a physical pointer to a virtual kernel pointer for /dev/mem
 * access
 */
#define xlate_dev_mem_ptr(p)	__va(p)

/*
 * Convert a virtual cached pointer to an uncached pointer
 */
#define xlate_dev_kmem_ptr(p)	p

#define memset_io(a,b,c)	memset((void *)(a),(b),(c))
#define memcpy_fromio(a,b,c)	memcpy((a),(void *)(b),(c))
#define memcpy_toio(a,b,c)	memcpy((void *)(a),(b),(c))

#define __raw_readb readb
#define __raw_readw readw
#define __raw_readl readl
#define __raw_writeb writeb
#define __raw_writew writew
#define __raw_writel writel

/*
 * Generic IO read/write.  These perform native-endian accesses.  Note
 * that some architectures will want to re-define __raw_{read,write}w.
 */
extern void __raw_writesb(void __iomem *addr, const void *data, int bytelen);
extern void __raw_writesw(void __iomem *addr, const void *data, int wordlen);
extern void __raw_writesl(void __iomem *addr, const void *data, int longlen);

extern void __raw_readsb(void __iomem *addr, void *data, int bytelen);
extern void __raw_readsw(void __iomem *addr, void *data, int wordlen);
extern void __raw_readsl(void __iomem *addr, void *data, int longlen);

#define readsb(p,d,l)           __raw_readsb((p),(d),(l))
#define readsw(p,d,l)           __raw_readsw((p),(d),(l))
#define readsl(p,d,l)           __raw_readsl((p),(d),(l))

#define writesb(p,d,l)          __raw_writesb((p),(d),(l))
#define writesw(p,d,l)          __raw_writesw((p),(d),(l))
#define writesl(p,d,l)          __raw_writesl((p),(d),(l))


static inline void __iomem *ioport_map(unsigned long port, unsigned int nr)
{
	return (void __iomem *) port;
}

static inline void ioport_unmap(void __iomem *p)
{
}

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_IO_H */
