/*
 *  linux/include/asm-c6x/io.h
 *
 *  Port on Texas Instruments TMS320C6x architecture
 *
 *  Copyright (C) 2004, 2009, 2010, 2011 Texas Instruments Incorporated
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

/*
 * traditional input/output functions (got from include/asm-generic/io.h)
 */

static inline u8 inb(unsigned long addr)
{
	return readb((volatile void __iomem *) addr);
}

static inline u16 inw(unsigned long addr)
{
	return readw((volatile void __iomem *) addr);
}

static inline u32 inl(unsigned long addr)
{
	return readl((volatile void __iomem *) addr);
}

static inline void outb(u8 b, unsigned long addr)
{
	writeb(b, (volatile void __iomem *) addr);
}

static inline void outw(u16 b, unsigned long addr)
{
	writew(b, (volatile void __iomem *) addr);
}

static inline void outl(u32 b, unsigned long addr)
{
	writel(b, (volatile void __iomem *) addr);
}

#define inb_p(addr)	inb(addr)
#define inw_p(addr)	inw(addr)
#define inl_p(addr)	inl(addr)
#define outb_p(x, addr)	outb((x), (addr))
#define outw_p(x, addr)	outw((x), (addr))
#define outl_p(x, addr)	outl((x), (addr))

static inline void insb(unsigned long addr, void *buffer, int count)
{
	if (count) {
		u8 *buf = buffer;
		do {
			u8 x = inb(addr);
			*buf++ = x;
		} while (--count);
	}
}

static inline void insw(unsigned long addr, void *buffer, int count)
{
	if (count) {
		u16 *buf = buffer;
		do {
			u16 x = inw(addr);
			*buf++ = x;
		} while (--count);
	}
}

static inline void insl(unsigned long addr, void *buffer, int count)
{
	if (count) {
		u32 *buf = buffer;
		do {
			u32 x = inl(addr);
			*buf++ = x;
		} while (--count);
	}
}

static inline void outsb(unsigned long addr, const void *buffer, int count)
{
	if (count) {
		const u8 *buf = buffer;
		do {
			outb(*buf++, addr);
		} while (--count);
	}
}

static inline void outsw(unsigned long addr, const void *buffer, int count)
{
	if (count) {
		const u16 *buf = buffer;
		do {
			outw(*buf++, addr);
		} while (--count);
	}
}

static inline void outsl(unsigned long addr, const void *buffer, int count)
{
	if (count) {
		const u32 *buf = buffer;
		do {
			outl(*buf++, addr);
		} while (--count);
	}
}

#define ioread8(addr)		readb(addr)
#define ioread16(addr)		readw(addr)
#define ioread32(addr)		readl(addr)

#define iowrite8(v, addr)	writeb((v), (addr))
#define iowrite16(v, addr)	writew((v), (addr))
#define iowrite32(v, addr)	writel((v), (addr))

#define ioread8_rep(p,d,c)      __raw_readsb(p,d,c)
#define ioread16_rep(p,d,c)     __raw_readsw(p,d,c)
#define ioread32_rep(p,d,c)     __raw_readsl(p,d,c)

#define iowrite8_rep(p,s,c)     __raw_writesb(p,s,c)
#define iowrite16_rep(p,s,c)    __raw_writesw(p,s,c)
#define iowrite32_rep(p,s,c)    __raw_writesl(p,s,c)

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

struct pci_dev;

extern void __iomem *pci_iomap(struct pci_dev *dev, int bar, unsigned long maxlen);
extern void pci_iounmap(struct pci_dev *dev, void __iomem *addr);

#endif /* __KERNEL__ */
#endif /* __ASM_C6X_IO_H */
