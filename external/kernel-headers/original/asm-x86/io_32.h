#ifndef _ASM_IO_H
#define _ASM_IO_H

#include <linux/string.h>
#include <linux/compiler.h>



 /*
  *  Bit simplified and optimized by Jan Hubicka
  *  Support of BIGMEM added by Gerhard Wichert, Siemens AG, July 1999.
  *
  *  isa_memset_io, isa_memcpy_fromio, isa_memcpy_toio added,
  *  isa_read[wl] and isa_write[wl] fixed
  *  - Arnaldo Carvalho de Melo <acme@conectiva.com.br>
  */

#define IO_SPACE_LIMIT 0xffff

#define XQUAD_PORTIO_BASE 0xfe400000
#define XQUAD_PORTIO_QUAD 0x40000  /* 256k per quad. */

#ifdef __KERNEL__

#include <asm-generic/iomap.h>

#include <linux/vmalloc.h>

#define xlate_dev_mem_ptr(p)	__va(p)

#define xlate_dev_kmem_ptr(p)	p

 
static inline unsigned long virt_to_phys(volatile void * address)
{
	return __pa(address);
}


static inline void * phys_to_virt(unsigned long address)
{
	return __va(address);
}

#define page_to_phys(page)    ((dma_addr_t)page_to_pfn(page) << PAGE_SHIFT)

extern void __iomem * __ioremap(unsigned long offset, unsigned long size, unsigned long flags);


static inline void __iomem * ioremap(unsigned long offset, unsigned long size)
{
	return __ioremap(offset, size, 0);
}

extern void __iomem * ioremap_nocache(unsigned long offset, unsigned long size);
extern void iounmap(volatile void __iomem *addr);

extern void *bt_ioremap(unsigned long offset, unsigned long size);
extern void bt_iounmap(void *addr, unsigned long size);
extern void __iomem *fix_ioremap(unsigned idx, unsigned long phys);

/* Use early IO mappings for DMI because it's initialized early */
#define dmi_ioremap bt_ioremap
#define dmi_iounmap bt_iounmap
#define dmi_alloc alloc_bootmem

#define isa_virt_to_bus virt_to_phys
#define isa_page_to_bus page_to_phys
#define isa_bus_to_virt phys_to_virt

#define virt_to_bus virt_to_phys
#define bus_to_virt phys_to_virt


static inline unsigned char readb(const volatile void __iomem *addr)
{
	return *(volatile unsigned char __force *) addr;
}
static inline unsigned short readw(const volatile void __iomem *addr)
{
	return *(volatile unsigned short __force *) addr;
}
static inline unsigned int readl(const volatile void __iomem *addr)
{
	return *(volatile unsigned int __force *) addr;
}
#define readb_relaxed(addr) readb(addr)
#define readw_relaxed(addr) readw(addr)
#define readl_relaxed(addr) readl(addr)
#define __raw_readb readb
#define __raw_readw readw
#define __raw_readl readl

static inline void writeb(unsigned char b, volatile void __iomem *addr)
{
	*(volatile unsigned char __force *) addr = b;
}
static inline void writew(unsigned short b, volatile void __iomem *addr)
{
	*(volatile unsigned short __force *) addr = b;
}
static inline void writel(unsigned int b, volatile void __iomem *addr)
{
	*(volatile unsigned int __force *) addr = b;
}
#define __raw_writeb writeb
#define __raw_writew writew
#define __raw_writel writel

#define mmiowb()

static inline void
memset_io(volatile void __iomem *addr, unsigned char val, int count)
{
	memset((void __force *)addr, val, count);
}

static inline void
memcpy_fromio(void *dst, const volatile void __iomem *src, int count)
{
	__memcpy(dst, (const void __force *)src, count);
}

static inline void
memcpy_toio(volatile void __iomem *dst, const void *src, int count)
{
	__memcpy((void __force *)dst, src, count);
}

#define __ISA_IO_base ((char __iomem *)(PAGE_OFFSET))

 
#if defined(CONFIG_X86_OOSTORE) || defined(CONFIG_X86_PPRO_FENCE)

static inline void flush_write_buffers(void)
{
	__asm__ __volatile__ ("lock; addl $0,0(%%esp)": : :"memory");
}

#else

#define flush_write_buffers() do { } while (0)

#endif

#endif /* __KERNEL__ */

static inline void native_io_delay(void)
{
	asm volatile("outb %%al,$0x80" : : : "memory");
}

#if defined(CONFIG_PARAVIRT)
#include <asm/paravirt.h>
#else

static inline void slow_down_io(void) {
	native_io_delay();
#ifdef REALLY_SLOW_IO
	native_io_delay();
	native_io_delay();
	native_io_delay();
#endif
}

#endif

#ifdef CONFIG_X86_NUMAQ
extern void *xquad_portio;    /* Where the IO area was mapped */
#define XQUAD_PORT_ADDR(port, quad) (xquad_portio + (XQUAD_PORTIO_QUAD*quad) + port)
#define __BUILDIO(bwl,bw,type) \
static inline void out##bwl##_quad(unsigned type value, int port, int quad) { \
	if (xquad_portio) \
		write##bwl(value, XQUAD_PORT_ADDR(port, quad)); \
	else \
		out##bwl##_local(value, port); \
} \
static inline void out##bwl(unsigned type value, int port) { \
	out##bwl##_quad(value, port, 0); \
} \
static inline unsigned type in##bwl##_quad(int port, int quad) { \
	if (xquad_portio) \
		return read##bwl(XQUAD_PORT_ADDR(port, quad)); \
	else \
		return in##bwl##_local(port); \
} \
static inline unsigned type in##bwl(int port) { \
	return in##bwl##_quad(port, 0); \
}
#else
#define __BUILDIO(bwl,bw,type) \
static inline void out##bwl(unsigned type value, int port) { \
	out##bwl##_local(value, port); \
} \
static inline unsigned type in##bwl(int port) { \
	return in##bwl##_local(port); \
}
#endif


#define BUILDIO(bwl,bw,type) \
static inline void out##bwl##_local(unsigned type value, int port) { \
	__asm__ __volatile__("out" #bwl " %" #bw "0, %w1" : : "a"(value), "Nd"(port)); \
} \
static inline unsigned type in##bwl##_local(int port) { \
	unsigned type value; \
	__asm__ __volatile__("in" #bwl " %w1, %" #bw "0" : "=a"(value) : "Nd"(port)); \
	return value; \
} \
static inline void out##bwl##_local_p(unsigned type value, int port) { \
	out##bwl##_local(value, port); \
	slow_down_io(); \
} \
static inline unsigned type in##bwl##_local_p(int port) { \
	unsigned type value = in##bwl##_local(port); \
	slow_down_io(); \
	return value; \
} \
__BUILDIO(bwl,bw,type) \
static inline void out##bwl##_p(unsigned type value, int port) { \
	out##bwl(value, port); \
	slow_down_io(); \
} \
static inline unsigned type in##bwl##_p(int port) { \
	unsigned type value = in##bwl(port); \
	slow_down_io(); \
	return value; \
} \
static inline void outs##bwl(int port, const void *addr, unsigned long count) { \
	__asm__ __volatile__("rep; outs" #bwl : "+S"(addr), "+c"(count) : "d"(port)); \
} \
static inline void ins##bwl(int port, void *addr, unsigned long count) { \
	__asm__ __volatile__("rep; ins" #bwl : "+D"(addr), "+c"(count) : "d"(port)); \
}

BUILDIO(b,b,char)
BUILDIO(w,w,short)
BUILDIO(l,,int)

#endif
