
#ifndef _ASM_IA64_SN_NODEPDA_H
#define _ASM_IA64_SN_NODEPDA_H


#include <asm/irq.h>
#include <asm/sn/arch.h>
#include <asm/sn/intr.h>
#include <asm/sn/bte.h>


struct phys_cpuid {
	short			nasid;
	char			subnode;
	char			slice;
};

struct nodepda_s {
	void 		*pdinfo;	/* Platform-dependent per-node info */

	/*
	 * The BTEs on this node are shared by the local cpus
	 */
	struct bteinfo_s	bte_if[MAX_BTES_PER_NODE];	/* Virtual Interface */
	struct timer_list	bte_recovery_timer;
	spinlock_t		bte_recovery_lock;

	/* 
	 * Array of pointers to the nodepdas for each node.
	 */
	struct nodepda_s	*pernode_pdaindr[MAX_COMPACT_NODES]; 

	/*
	 * Array of physical cpu identifiers. Indexed by cpuid.
	 */
	struct phys_cpuid	phys_cpuid[NR_CPUS];
	spinlock_t		ptc_lock ____cacheline_aligned_in_smp;
};

typedef struct nodepda_s nodepda_t;


DECLARE_PER_CPU(struct nodepda_s *, __sn_nodepda);
#define sn_nodepda		(__get_cpu_var(__sn_nodepda))
#define	NODEPDA(cnodeid)	(sn_nodepda->pernode_pdaindr[cnodeid])

#define is_headless_node(cnodeid)	(nr_cpus_node(cnodeid) == 0)

#endif /* _ASM_IA64_SN_NODEPDA_H */
