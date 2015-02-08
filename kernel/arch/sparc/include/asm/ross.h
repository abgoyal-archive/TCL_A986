

#ifndef _SPARC_ROSS_H
#define _SPARC_ROSS_H

#include <asm/asi.h>
#include <asm/page.h>



#define HYPERSPARC_CWENABLE   0x00200000
#define HYPERSPARC_SBENABLE   0x00100000
#define HYPERSPARC_WBENABLE   0x00080000
#define HYPERSPARC_MIDMASK    0x00078000
#define HYPERSPARC_BMODE      0x00004000
#define HYPERSPARC_ACENABLE   0x00002000
#define HYPERSPARC_CSIZE      0x00001000
#define HYPERSPARC_MRFLCT     0x00000800
#define HYPERSPARC_CMODE      0x00000400
#define HYPERSPARC_CENABLE    0x00000100
#define HYPERSPARC_NFAULT     0x00000002
#define HYPERSPARC_MENABLE    0x00000001



#define HYPERSPARC_ICCR_FTD     0x00000002
#define HYPERSPARC_ICCR_ICE     0x00000001

#ifndef __ASSEMBLY__

static inline unsigned int get_ross_icr(void)
{
	unsigned int icreg;

	__asm__ __volatile__(".word 0x8347c000\n\t" /* rd %iccr, %g1 */
			     "mov %%g1, %0\n\t"
			     : "=r" (icreg)
			     : /* no inputs */
			     : "g1", "memory");

	return icreg;
}

static inline void put_ross_icr(unsigned int icreg)
{
	__asm__ __volatile__("or %%g0, %0, %%g1\n\t"
			     ".word 0xbf806000\n\t" /* wr %g1, 0x0, %iccr */
			     "nop\n\t"
			     "nop\n\t"
			     "nop\n\t"
			     : /* no outputs */
			     : "r" (icreg)
			     : "g1", "memory");

	return;
}

/* HyperSparc specific cache flushing. */

/* This is for the on-chip instruction cache. */
static inline void hyper_flush_whole_icache(void)
{
	__asm__ __volatile__("sta %%g0, [%%g0] %0\n\t"
			     : /* no outputs */
			     : "i" (ASI_M_FLUSH_IWHOLE)
			     : "memory");
	return;
}

extern int vac_cache_size;
extern int vac_line_size;

static inline void hyper_clear_all_tags(void)
{
	unsigned long addr;

	for(addr = 0; addr < vac_cache_size; addr += vac_line_size)
		__asm__ __volatile__("sta %%g0, [%0] %1\n\t"
				     : /* no outputs */
				     : "r" (addr), "i" (ASI_M_DATAC_TAG)
				     : "memory");
}

static inline void hyper_flush_unconditional_combined(void)
{
	unsigned long addr;

	for (addr = 0; addr < vac_cache_size; addr += vac_line_size)
		__asm__ __volatile__("sta %%g0, [%0] %1\n\t"
				     : /* no outputs */
				     : "r" (addr), "i" (ASI_M_FLUSH_CTX)
				     : "memory");
}

static inline void hyper_flush_cache_user(void)
{
	unsigned long addr;

	for (addr = 0; addr < vac_cache_size; addr += vac_line_size)
		__asm__ __volatile__("sta %%g0, [%0] %1\n\t"
				     : /* no outputs */
				     : "r" (addr), "i" (ASI_M_FLUSH_USER)
				     : "memory");
}

static inline void hyper_flush_cache_page(unsigned long page)
{
	unsigned long end;

	page &= PAGE_MASK;
	end = page + PAGE_SIZE;
	while (page < end) {
		__asm__ __volatile__("sta %%g0, [%0] %1\n\t"
				     : /* no outputs */
				     : "r" (page), "i" (ASI_M_FLUSH_PAGE)
				     : "memory");
		page += vac_line_size;
	}
}

#endif /* !(__ASSEMBLY__) */

#endif /* !(_SPARC_ROSS_H) */
