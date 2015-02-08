

#ifndef _ASM_MICROBLAZE_CACHEFLUSH_H
#define _ASM_MICROBLAZE_CACHEFLUSH_H

/* Somebody depends on this; sigh... */
#include <linux/mm.h>

/* Look at Documentation/cachetlb.txt */


struct scache {
	/* icache */
	void (*ie)(void); /* enable */
	void (*id)(void); /* disable */
	void (*ifl)(void); /* flush */
	void (*iflr)(unsigned long a, unsigned long b);
	void (*iin)(void); /* invalidate */
	void (*iinr)(unsigned long a, unsigned long b);
	/* dcache */
	void (*de)(void); /* enable */
	void (*dd)(void); /* disable */
	void (*dfl)(void); /* flush */
	void (*dflr)(unsigned long a, unsigned long b);
	void (*din)(void); /* invalidate */
	void (*dinr)(unsigned long a, unsigned long b);
};

/* microblaze cache */
extern struct scache *mbc;

void microblaze_cache_init(void);

#define enable_icache()					mbc->ie();
#define disable_icache()				mbc->id();
#define flush_icache()					mbc->ifl();
#define flush_icache_range(start, end)			mbc->iflr(start, end);
#define invalidate_icache()				mbc->iin();
#define invalidate_icache_range(start, end)		mbc->iinr(start, end);


#define flush_icache_user_range(vma, pg, adr, len)	flush_icache();
#define flush_icache_page(vma, pg)			do { } while (0)

#define enable_dcache()					mbc->de();
#define disable_dcache()				mbc->dd();
/* FIXME for LL-temac driver */
#define invalidate_dcache()				mbc->din();
#define invalidate_dcache_range(start, end)		mbc->dinr(start, end);
#define flush_dcache()					mbc->dfl();
#define flush_dcache_range(start, end)			mbc->dflr(start, end);

#define ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE 0
/* D-cache aliasing problem can't happen - cache is between MMU and ram */
#define flush_dcache_page(page)			do { } while (0)
#define flush_dcache_mmap_lock(mapping)		do { } while (0)
#define flush_dcache_mmap_unlock(mapping)	do { } while (0)


#define flush_cache_dup_mm(mm)				do { } while (0)
#define flush_cache_vmap(start, end)			do { } while (0)
#define flush_cache_vunmap(start, end)			do { } while (0)
#define flush_cache_mm(mm)			do { } while (0)
#define flush_cache_page(vma, vmaddr, pfn)	do { } while (0)

/* MS: kgdb code use this macro, wrong len with FLASH */
#if 0
#define flush_cache_range(vma, start, len)	{	\
	flush_icache_range((unsigned) (start), (unsigned) (start) + (len)); \
	flush_dcache_range((unsigned) (start), (unsigned) (start) + (len)); \
}
#endif

#define flush_cache_range(vma, start, len) do { } while (0)

#define copy_to_user_page(vma, page, vaddr, dst, src, len)		\
do {									\
	memcpy((dst), (src), (len));					\
	flush_icache_range((unsigned) (dst), (unsigned) (dst) + (len));	\
} while (0)

#define copy_from_user_page(vma, page, vaddr, dst, src, len)		\
do {									\
	memcpy((dst), (src), (len));					\
} while (0)

#endif /* _ASM_MICROBLAZE_CACHEFLUSH_H */
