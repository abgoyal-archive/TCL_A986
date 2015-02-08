
#ifndef _ASM_SN_SN0_ARCH_H
#define _ASM_SN_SN0_ARCH_H


#ifndef SN0XXL  /* 128 cpu SMP max */
#define MAX_COMPACT_NODES       64

#define MAXCPUS                 128

#else /* SN0XXL system */

#define MAX_COMPACT_NODES       128
#define MAXCPUS                 256

#endif /* SN0XXL */

#define MAX_NASIDS		256

#define	MAX_REGIONS		64
#define MAX_NONPREMIUM_REGIONS  16
#define MAX_PREMIUM_REGIONS     MAX_REGIONS

#define MAX_PARTITIONS		MAX_REGIONS

#define NASID_MASK_BYTES	((MAX_NASIDS + 7) / 8)

#ifdef CONFIG_SGI_SN_N_MODE
#define MAX_MEM_SLOTS   16                      /* max slots per node */
#else /* !CONFIG_SGI_SN_N_MODE, assume CONFIG_SGI_SN_M_MODE */
#define MAX_MEM_SLOTS   32                      /* max slots per node */
#endif /* CONFIG_SGI_SN_M_MODE */

#define SLOT_SHIFT      	(27)
#define SLOT_MIN_MEM_SIZE	(32*1024*1024)

#define CPUS_PER_NODE		2	/* CPUs on a single hub */
#define CPUS_PER_NODE_SHFT	1	/* Bits to shift in the node number */
#define CPUS_PER_SUBNODE	2	/* CPUs on a single hub PI */

#endif /* _ASM_SN_SN0_ARCH_H */
