
#ifndef _ASMARM_CACHEFLUSH_H
#define _ASMARM_CACHEFLUSH_H

#include <linux/mm.h>

#include <asm/glue.h>
#include <asm/shmparam.h>
#include <asm/cachetype.h>
#include <asm/outercache.h>

#define CACHE_COLOUR(vaddr)	((vaddr & (SHMLBA - 1)) >> PAGE_SHIFT)

#undef _CACHE
#undef MULTI_CACHE

#if defined(CONFIG_CPU_CACHE_V3)
# ifdef _CACHE
#  define MULTI_CACHE 1
# else
#  define _CACHE v3
# endif
#endif

#if defined(CONFIG_CPU_CACHE_V4)
# ifdef _CACHE
#  define MULTI_CACHE 1
# else
#  define _CACHE v4
# endif
#endif

#if defined(CONFIG_CPU_ARM920T) || defined(CONFIG_CPU_ARM922T) || \
    defined(CONFIG_CPU_ARM925T) || defined(CONFIG_CPU_ARM1020) || \
    defined(CONFIG_CPU_ARM1026)
# define MULTI_CACHE 1
#endif

#if defined(CONFIG_CPU_FA526)
# ifdef _CACHE
#  define MULTI_CACHE 1
# else
#  define _CACHE fa
# endif
#endif

#if defined(CONFIG_CPU_ARM926T)
# ifdef _CACHE
#  define MULTI_CACHE 1
# else
#  define _CACHE arm926
# endif
#endif

#if defined(CONFIG_CPU_ARM940T)
# ifdef _CACHE
#  define MULTI_CACHE 1
# else
#  define _CACHE arm940
# endif
#endif

#if defined(CONFIG_CPU_ARM946E)
# ifdef _CACHE
#  define MULTI_CACHE 1
# else
#  define _CACHE arm946
# endif
#endif

#if defined(CONFIG_CPU_CACHE_V4WB)
# ifdef _CACHE
#  define MULTI_CACHE 1
# else
#  define _CACHE v4wb
# endif
#endif

#if defined(CONFIG_CPU_XSCALE)
# ifdef _CACHE
#  define MULTI_CACHE 1
# else
#  define _CACHE xscale
# endif
#endif

#if defined(CONFIG_CPU_XSC3)
# ifdef _CACHE
#  define MULTI_CACHE 1
# else
#  define _CACHE xsc3
# endif
#endif

#if defined(CONFIG_CPU_MOHAWK)
# ifdef _CACHE
#  define MULTI_CACHE 1
# else
#  define _CACHE mohawk
# endif
#endif

#if defined(CONFIG_CPU_FEROCEON)
# define MULTI_CACHE 1
#endif

#if defined(CONFIG_CPU_V6)
//# ifdef _CACHE
#  define MULTI_CACHE 1
//# else
//#  define _CACHE v6
//# endif
#endif

#if defined(CONFIG_CPU_V7)
//# ifdef _CACHE
#  define MULTI_CACHE 1
//# else
//#  define _CACHE v7
//# endif
#endif

#if !defined(_CACHE) && !defined(MULTI_CACHE)
#error Unknown cache maintainence model
#endif

#define PG_dcache_dirty PG_arch_1


struct cpu_cache_fns {
	void (*flush_kern_all)(void);
	void (*flush_user_all)(void);
	void (*flush_user_range)(unsigned long, unsigned long, unsigned int);

	void (*coherent_kern_range)(unsigned long, unsigned long);
	void (*coherent_user_range)(unsigned long, unsigned long);
	void (*flush_kern_dcache_area)(void *, size_t);

	void (*dma_map_area)(const void *, size_t, int);
	void (*dma_unmap_area)(const void *, size_t, int);

	void (*dma_flush_range)(const void *, const void *);
};

#ifdef MULTI_CACHE

extern struct cpu_cache_fns cpu_cache;

#define __cpuc_flush_kern_all		cpu_cache.flush_kern_all
#define __cpuc_flush_user_all		cpu_cache.flush_user_all
#define __cpuc_flush_user_range		cpu_cache.flush_user_range
#define __cpuc_coherent_kern_range	cpu_cache.coherent_kern_range
#define __cpuc_coherent_user_range	cpu_cache.coherent_user_range
#define __cpuc_flush_dcache_area	cpu_cache.flush_kern_dcache_area

#define dmac_map_area			cpu_cache.dma_map_area
#define dmac_unmap_area		cpu_cache.dma_unmap_area
#define dmac_flush_range		cpu_cache.dma_flush_range

#else

#define __cpuc_flush_kern_all		__glue(_CACHE,_flush_kern_cache_all)
#define __cpuc_flush_user_all		__glue(_CACHE,_flush_user_cache_all)
#define __cpuc_flush_user_range		__glue(_CACHE,_flush_user_cache_range)
#define __cpuc_coherent_kern_range	__glue(_CACHE,_coherent_kern_range)
#define __cpuc_coherent_user_range	__glue(_CACHE,_coherent_user_range)
#define __cpuc_flush_dcache_area	__glue(_CACHE,_flush_kern_dcache_area)

extern void __cpuc_flush_kern_all(void);
extern void __cpuc_flush_user_all(void);
extern void __cpuc_flush_user_range(unsigned long, unsigned long, unsigned int);
extern void __cpuc_coherent_kern_range(unsigned long, unsigned long);
extern void __cpuc_coherent_user_range(unsigned long, unsigned long);
extern void __cpuc_flush_dcache_area(void *, size_t);

#define dmac_map_area			__glue(_CACHE,_dma_map_area)
#define dmac_unmap_area		__glue(_CACHE,_dma_unmap_area)
#define dmac_flush_range		__glue(_CACHE,_dma_flush_range)

extern void dmac_map_area(const void *, size_t, int);
extern void dmac_unmap_area(const void *, size_t, int);
extern void dmac_flush_range(const void *, const void *);

#endif

extern void copy_to_user_page(struct vm_area_struct *, struct page *,
	unsigned long, void *, const void *, unsigned long);
#define copy_from_user_page(vma, page, vaddr, dst, src, len) \
	do {							\
		memcpy(dst, src, len);				\
	} while (0)

#define flush_cache_all()		__cpuc_flush_kern_all()

static inline void vivt_flush_cache_mm(struct mm_struct *mm)
{
	if (cpumask_test_cpu(smp_processor_id(), mm_cpumask(mm)))
		__cpuc_flush_user_all();
}

static inline void
vivt_flush_cache_range(struct vm_area_struct *vma, unsigned long start, unsigned long end)
{
	if (cpumask_test_cpu(smp_processor_id(), mm_cpumask(vma->vm_mm)))
		__cpuc_flush_user_range(start & PAGE_MASK, PAGE_ALIGN(end),
					vma->vm_flags);
}

static inline void
vivt_flush_cache_page(struct vm_area_struct *vma, unsigned long user_addr, unsigned long pfn)
{
	if (cpumask_test_cpu(smp_processor_id(), mm_cpumask(vma->vm_mm))) {
		unsigned long addr = user_addr & PAGE_MASK;
		__cpuc_flush_user_range(addr, addr + PAGE_SIZE, vma->vm_flags);
	}
}

#ifndef CONFIG_CPU_CACHE_VIPT
#define flush_cache_mm(mm) \
		vivt_flush_cache_mm(mm)
#define flush_cache_range(vma,start,end) \
		vivt_flush_cache_range(vma,start,end)
#define flush_cache_page(vma,addr,pfn) \
		vivt_flush_cache_page(vma,addr,pfn)
#else
extern void flush_cache_mm(struct mm_struct *mm);
extern void flush_cache_range(struct vm_area_struct *vma, unsigned long start, unsigned long end);
extern void flush_cache_page(struct vm_area_struct *vma, unsigned long user_addr, unsigned long pfn);
#endif

#define flush_cache_dup_mm(mm) flush_cache_mm(mm)

#define flush_cache_user_range(start,end) \
	__cpuc_coherent_user_range((start) & PAGE_MASK, PAGE_ALIGN(end))

#define flush_icache_range(s,e)		__cpuc_coherent_kern_range(s,e)

#define clean_dcache_area(start,size)	cpu_dcache_clean_area(start, size)

#define ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE 1
extern void flush_dcache_page(struct page *);

static inline void __flush_icache_all(void)
{
#ifdef CONFIG_ARM_ERRATA_411920
	extern void v6_icache_inval_all(void);
	v6_icache_inval_all();
#elif defined(CONFIG_SMP) && __LINUX_ARM_ARCH__ >= 7
	asm("mcr	p15, 0, %0, c7, c1, 0	@ invalidate I-cache inner shareable\n"
	    :
	    : "r" (0));
#else
	asm("mcr	p15, 0, %0, c7, c5, 0	@ invalidate I-cache\n"
	    :
	    : "r" (0));
#endif
}
static inline void flush_kernel_vmap_range(void *addr, int size)
{
	if ((cache_is_vivt() || cache_is_vipt_aliasing()))
	  __cpuc_flush_dcache_area(addr, (size_t)size);
}
static inline void invalidate_kernel_vmap_range(void *addr, int size)
{
	if ((cache_is_vivt() || cache_is_vipt_aliasing()))
	  __cpuc_flush_dcache_area(addr, (size_t)size);
}

#define ARCH_HAS_FLUSH_ANON_PAGE
static inline void flush_anon_page(struct vm_area_struct *vma,
			 struct page *page, unsigned long vmaddr)
{
	extern void __flush_anon_page(struct vm_area_struct *vma,
				struct page *, unsigned long);
	if (PageAnon(page))
		__flush_anon_page(vma, page, vmaddr);
}

#define ARCH_HAS_FLUSH_KERNEL_DCACHE_PAGE
static inline void flush_kernel_dcache_page(struct page *page)
{
	/* highmem pages are always flushed upon kunmap already */
	if ((cache_is_vivt() || cache_is_vipt_aliasing()) && !PageHighMem(page))
		__cpuc_flush_dcache_area(page_address(page), PAGE_SIZE);
}

#define flush_dcache_mmap_lock(mapping) \
	spin_lock_irq(&(mapping)->tree_lock)
#define flush_dcache_mmap_unlock(mapping) \
	spin_unlock_irq(&(mapping)->tree_lock)

#define flush_icache_user_range(vma,page,addr,len) \
	flush_dcache_page(page)

#define flush_icache_page(vma,page)	do { } while (0)

static inline void flush_cache_vmap(unsigned long start, unsigned long end)
{
	if (!cache_is_vipt_nonaliasing())
		flush_cache_all();
	else
		/*
		 * set_pte_at() called from vmap_pte_range() does not
		 * have a DSB after cleaning the cache line.
		 */
		dsb();
}

static inline void flush_cache_vunmap(unsigned long start, unsigned long end)
{
	if (!cache_is_vipt_nonaliasing())
		flush_cache_all();
}

#endif
