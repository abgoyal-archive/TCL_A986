

#ifndef _ASM_IA64_XEN_PAGE_H
#define _ASM_IA64_XEN_PAGE_H

#define INVALID_P2M_ENTRY	(~0UL)

static inline unsigned long mfn_to_pfn(unsigned long mfn)
{
	return mfn;
}

static inline unsigned long pfn_to_mfn(unsigned long pfn)
{
	return pfn;
}

#define phys_to_machine_mapping_valid(_x)	(1)

static inline void *mfn_to_virt(unsigned long mfn)
{
	return __va(mfn << PAGE_SHIFT);
}

static inline unsigned long virt_to_mfn(void *virt)
{
	return __pa(virt) >> PAGE_SHIFT;
}

/* for tpmfront.c */
static inline unsigned long virt_to_machine(void *virt)
{
	return __pa(virt);
}

static inline void set_phys_to_machine(unsigned long pfn, unsigned long mfn)
{
	/* nothing */
}

#define pte_mfn(_x)	pte_pfn(_x)
#define mfn_pte(_x, _y)	__pte_ma(0)		/* unmodified use */
#define __pte_ma(_x)	((pte_t) {(_x)})        /* unmodified use */

#endif /* _ASM_IA64_XEN_PAGE_H */
