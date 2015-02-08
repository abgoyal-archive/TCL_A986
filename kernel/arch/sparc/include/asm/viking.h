
#ifndef _SPARC_VIKING_H
#define _SPARC_VIKING_H

#include <asm/asi.h>
#include <asm/mxcc.h>
#include <asm/pgtsrmmu.h>


#define VIKING_MMUENABLE    0x00000001
#define VIKING_NOFAULT      0x00000002
#define VIKING_PSO          0x00000080
#define VIKING_DCENABLE     0x00000100   /* Enable data cache */
#define VIKING_ICENABLE     0x00000200   /* Enable instruction cache */
#define VIKING_SBENABLE     0x00000400   /* Enable store buffer */
#define VIKING_MMODE        0x00000800   /* MBUS mode */
#define VIKING_PCENABLE     0x00001000   /* Enable parity checking */
#define VIKING_BMODE        0x00002000   
#define VIKING_SPENABLE     0x00004000   /* Enable bus cache snooping */
#define VIKING_ACENABLE     0x00008000   /* Enable alternate caching */
#define VIKING_TCENABLE     0x00010000   /* Enable table-walks to be cached */
#define VIKING_DPENABLE     0x00040000   /* Enable the data prefetcher */

#define VIKING_ACTION_MIX   0x00001000   /* Enable multiple instructions */

#define VIKING_PTAG_VALID   0x01000000   /* Cache block is valid */
#define VIKING_PTAG_DIRTY   0x00010000   /* Block has been modified */
#define VIKING_PTAG_SHARED  0x00000100   /* Shared with some other cache */

#ifndef __ASSEMBLY__

static inline void viking_flush_icache(void)
{
	__asm__ __volatile__("sta %%g0, [%%g0] %0\n\t"
			     : /* no outputs */
			     : "i" (ASI_M_IC_FLCLEAR)
			     : "memory");
}

static inline void viking_flush_dcache(void)
{
	__asm__ __volatile__("sta %%g0, [%%g0] %0\n\t"
			     : /* no outputs */
			     : "i" (ASI_M_DC_FLCLEAR)
			     : "memory");
}

static inline void viking_unlock_icache(void)
{
	__asm__ __volatile__("sta %%g0, [%0] %1\n\t"
			     : /* no outputs */
			     : "r" (0x80000000), "i" (ASI_M_IC_FLCLEAR)
			     : "memory");
}

static inline void viking_unlock_dcache(void)
{
	__asm__ __volatile__("sta %%g0, [%0] %1\n\t"
			     : /* no outputs */
			     : "r" (0x80000000), "i" (ASI_M_DC_FLCLEAR)
			     : "memory");
}

static inline void viking_set_bpreg(unsigned long regval)
{
	__asm__ __volatile__("sta %0, [%%g0] %1\n\t"
			     : /* no outputs */
			     : "r" (regval), "i" (ASI_M_ACTION)
			     : "memory");
}

static inline unsigned long viking_get_bpreg(void)
{
	unsigned long regval;

	__asm__ __volatile__("lda [%%g0] %1, %0\n\t"
			     : "=r" (regval)
			     : "i" (ASI_M_ACTION));
	return regval;
}

static inline void viking_get_dcache_ptag(int set, int block,
					      unsigned long *data)
{
	unsigned long ptag = ((set & 0x7f) << 5) | ((block & 0x3) << 26) |
			     0x80000000;
	unsigned long info, page;

	__asm__ __volatile__ ("ldda [%2] %3, %%g2\n\t"
			      "or %%g0, %%g2, %0\n\t"
			      "or %%g0, %%g3, %1\n\t"
			      : "=r" (info), "=r" (page)
			      : "r" (ptag), "i" (ASI_M_DATAC_TAG)
			      : "g2", "g3");
	data[0] = info;
	data[1] = page;
}

static inline void viking_mxcc_turn_off_parity(unsigned long *mregp,
						   unsigned long *mxcc_cregp)
{
	unsigned long mreg = *mregp;
	unsigned long mxcc_creg = *mxcc_cregp;

	mreg &= ~(VIKING_PCENABLE);
	mxcc_creg &= ~(MXCC_CTL_PARE);

	__asm__ __volatile__ ("set 1f, %%g2\n\t"
			      "andcc %%g2, 4, %%g0\n\t"
			      "bne 2f\n\t"
			      " nop\n"
			      "1:\n\t"
			      "sta %0, [%%g0] %3\n\t"
			      "sta %1, [%2] %4\n\t"
			      "b 1f\n\t"
			      " nop\n\t"
			      "nop\n"
			      "2:\n\t"
			      "sta %0, [%%g0] %3\n\t"
			      "sta %1, [%2] %4\n"
			      "1:\n\t"
			      : /* no output */
			      : "r" (mreg), "r" (mxcc_creg),
			        "r" (MXCC_CREG), "i" (ASI_M_MMUREGS),
			        "i" (ASI_M_MXCC)
			      : "g2", "memory", "cc");
	*mregp = mreg;
	*mxcc_cregp = mxcc_creg;
}

static inline unsigned long viking_hwprobe(unsigned long vaddr)
{
	unsigned long val;

	vaddr &= PAGE_MASK;
	/* Probe all MMU entries. */
	__asm__ __volatile__("lda [%1] %2, %0\n\t"
			     : "=r" (val)
			     : "r" (vaddr | 0x400), "i" (ASI_M_FLUSH_PROBE));
	if (!val)
		return 0;

	/* Probe region. */
	__asm__ __volatile__("lda [%1] %2, %0\n\t"
			     : "=r" (val)
			     : "r" (vaddr | 0x200), "i" (ASI_M_FLUSH_PROBE));
	if ((val & SRMMU_ET_MASK) == SRMMU_ET_PTE) {
		vaddr &= ~SRMMU_PGDIR_MASK;
		vaddr >>= PAGE_SHIFT;
		return val | (vaddr << 8);
	}

	/* Probe segment. */
	__asm__ __volatile__("lda [%1] %2, %0\n\t"
			     : "=r" (val)
			     : "r" (vaddr | 0x100), "i" (ASI_M_FLUSH_PROBE));
	if ((val & SRMMU_ET_MASK) == SRMMU_ET_PTE) {
		vaddr &= ~SRMMU_REAL_PMD_MASK;
		vaddr >>= PAGE_SHIFT;
		return val | (vaddr << 8);
	}

	/* Probe page. */
	__asm__ __volatile__("lda [%1] %2, %0\n\t"
			     : "=r" (val)
			     : "r" (vaddr), "i" (ASI_M_FLUSH_PROBE));
	return val;
}

#endif /* !__ASSEMBLY__ */

#endif /* !(_SPARC_VIKING_H) */
