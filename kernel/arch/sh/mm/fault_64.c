
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <asm/system.h>
#include <asm/tlb.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/pgalloc.h>
#include <asm/mmu_context.h>
#include <cpu/registers.h>

/* Callable from fault.c, so not static */
inline void __do_tlb_refill(unsigned long address,
                            unsigned long long is_text_not_data, pte_t *pte)
{
	unsigned long long ptel;
	unsigned long long pteh=0;
	struct tlb_info *tlbp;
	unsigned long long next;

	/* Get PTEL first */
	ptel = pte_val(*pte);

	/*
	 * Set PTEH register
	 */
	pteh = neff_sign_extend(address & MMU_VPN_MASK);

	/* Set the ASID. */
	pteh |= get_asid() << PTEH_ASID_SHIFT;
	pteh |= PTEH_VALID;

	/* Set PTEL register, set_pte has performed the sign extension */
	ptel &= _PAGE_FLAGS_HARDWARE_MASK; /* drop software flags */

	tlbp = is_text_not_data ? &(cpu_data->itlb) : &(cpu_data->dtlb);
	next = tlbp->next;
	__flush_tlb_slot(next);
	asm volatile ("putcfg %0,1,%2\n\n\t"
		      "putcfg %0,0,%1\n"
		      :  : "r" (next), "r" (pteh), "r" (ptel) );

	next += TLB_STEP;
	if (next > tlbp->last) next = tlbp->first;
	tlbp->next = next;

}

static int handle_vmalloc_fault(struct mm_struct *mm,
				unsigned long protection_flags,
                                unsigned long long textaccess,
				unsigned long address)
{
	pgd_t *dir;
	pud_t *pud;
	pmd_t *pmd;
	static pte_t *pte;
	pte_t entry;

	dir = pgd_offset_k(address);

	pud = pud_offset(dir, address);
	if (pud_none_or_clear_bad(pud))
		return 0;

	pmd = pmd_offset(pud, address);
	if (pmd_none_or_clear_bad(pmd))
		return 0;

	pte = pte_offset_kernel(pmd, address);
	entry = *pte;

	if (pte_none(entry) || !pte_present(entry))
		return 0;
	if ((pte_val(entry) & protection_flags) != protection_flags)
		return 0;

        __do_tlb_refill(address, textaccess, pte);

	return 1;
}

static int handle_tlbmiss(struct mm_struct *mm,
			  unsigned long long protection_flags,
			  unsigned long long textaccess,
			  unsigned long address)
{
	pgd_t *dir;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
	pte_t entry;

	/* NB. The PGD currently only contains a single entry - there is no
	   page table tree stored for the top half of the address space since
	   virtual pages in that region should never be mapped in user mode.
	   (In kernel mode, the only things in that region are the 512Mb super
	   page (locked in), and vmalloc (modules) +  I/O device pages (handled
	   by handle_vmalloc_fault), so no PGD for the upper half is required
	   by kernel mode either).

	   See how mm->pgd is allocated and initialised in pgd_alloc to see why
	   the next test is necessary.  - RPC */
	if (address >= (unsigned long) TASK_SIZE)
		/* upper half - never has page table entries. */
		return 0;

	dir = pgd_offset(mm, address);
	if (pgd_none(*dir) || !pgd_present(*dir))
		return 0;
	if (!pgd_present(*dir))
		return 0;

	pud = pud_offset(dir, address);
	if (pud_none(*pud) || !pud_present(*pud))
		return 0;

	pmd = pmd_offset(pud, address);
	if (pmd_none(*pmd) || !pmd_present(*pmd))
		return 0;

	pte = pte_offset_kernel(pmd, address);
	entry = *pte;

	if (pte_none(entry) || !pte_present(entry))
		return 0;

	/*
	 * If the page doesn't have sufficient protection bits set to
	 * service the kind of fault being handled, there's not much
	 * point doing the TLB refill.  Punt the fault to the general
	 * handler.
	 */
	if ((pte_val(entry) & protection_flags) != protection_flags)
		return 0;

        __do_tlb_refill(address, textaccess, pte);

	return 1;
}

struct expevt_lookup {
	unsigned short protection_flags[8];
	unsigned char  is_text_access[8];
	unsigned char  is_write_access[8];
};

#define PRU (1<<9)
#define PRW (1<<8)
#define PRX (1<<7)
#define PRR (1<<6)

#define DIRTY (_PAGE_DIRTY | _PAGE_ACCESSED)
#define YOUNG (_PAGE_ACCESSED)

static struct expevt_lookup expevt_lookup_table = {
	.protection_flags = {PRX, PRX, 0, 0, PRR, PRR, PRW, PRW},
	.is_text_access   = {1,   1,   0, 0, 0,   0,   0,   0}
};

asmlinkage int do_fast_page_fault(unsigned long long ssr_md,
				  unsigned long long expevt,
			          unsigned long address)
{
	struct task_struct *tsk;
	struct mm_struct *mm;
	unsigned long long textaccess;
	unsigned long long protection_flags;
	unsigned long long index;
	unsigned long long expevt4;

	/* The next few lines implement a way of hashing EXPEVT into a
	 * small array index which can be used to lookup parameters
	 * specific to the type of TLBMISS being handled.
	 *
	 * Note:
	 *	ITLBMISS has EXPEVT==0xa40
	 *	RTLBMISS has EXPEVT==0x040
	 *	WTLBMISS has EXPEVT==0x060
	 */
	expevt4 = (expevt >> 4);
	/* TODO : xor ssr_md into this expression too. Then we can check
	 * that PRU is set when it needs to be. */
	index = expevt4 ^ (expevt4 >> 5);
	index &= 7;
	protection_flags = expevt_lookup_table.protection_flags[index];
	textaccess       = expevt_lookup_table.is_text_access[index];

	/* SIM
	 * Note this is now called with interrupts still disabled
	 * This is to cope with being called for a missing IO port
	 * address with interrupts disabled. This should be fixed as
	 * soon as we have a better 'fast path' miss handler.
	 *
	 * Plus take care how you try and debug this stuff.
	 * For example, writing debug data to a port which you
	 * have just faulted on is not going to work.
	 */

	tsk = current;
	mm = tsk->mm;

	if ((address >= VMALLOC_START && address < VMALLOC_END) ||
	    (address >= IOBASE_VADDR  && address < IOBASE_END)) {
		if (ssr_md)
			/*
			 * Process-contexts can never have this address
			 * range mapped
			 */
			if (handle_vmalloc_fault(mm, protection_flags,
						 textaccess, address))
				return 1;
	} else if (!in_interrupt() && mm) {
		if (handle_tlbmiss(mm, protection_flags, textaccess, address))
			return 1;
	}

	return 0;
}
