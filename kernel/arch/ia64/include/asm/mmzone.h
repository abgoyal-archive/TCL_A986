
#ifndef _ASM_IA64_MMZONE_H
#define _ASM_IA64_MMZONE_H

#include <linux/numa.h>
#include <asm/page.h>
#include <asm/meminit.h>

#ifdef CONFIG_NUMA

static inline int pfn_to_nid(unsigned long pfn)
{
	extern int paddr_to_nid(unsigned long);
	int nid = paddr_to_nid(pfn << PAGE_SHIFT);
	if (nid < 0)
		return 0;
	else
		return nid;
}

#ifdef CONFIG_IA64_DIG /* DIG systems are small */
# define MAX_PHYSNODE_ID	8
# define NR_NODE_MEMBLKS	(MAX_NUMNODES * 8)
#else /* sn2 is the biggest case, so we use that if !DIG */
# define MAX_PHYSNODE_ID	2048
# define NR_NODE_MEMBLKS	(MAX_NUMNODES * 4)
#endif

#else /* CONFIG_NUMA */
# define NR_NODE_MEMBLKS	(MAX_NUMNODES * 4)
#endif /* CONFIG_NUMA */

#endif /* _ASM_IA64_MMZONE_H */
