

#ifndef _ASM_MMZONE_H_
#define _ASM_MMZONE_H_

#include <asm/smp.h>

#ifdef CONFIG_DISCONTIGMEM

extern struct pglist_data *node_data[];
#define NODE_DATA(nid)		(node_data[nid])

#define node_localnr(pfn, nid)	((pfn) - NODE_DATA(nid)->node_start_pfn)
#define node_start_pfn(nid)	(NODE_DATA(nid)->node_start_pfn)
#define node_end_pfn(nid)						\
({									\
	pg_data_t *__pgdat = NODE_DATA(nid);				\
	__pgdat->node_start_pfn + __pgdat->node_spanned_pages - 1;	\
})

#define pmd_page(pmd)		(pfn_to_page(pmd_val(pmd) >> PAGE_SHIFT))
#if 1	/* M32R_FIXME */
#define pfn_valid(pfn)	(1)
#else
#define pfn_valid(pfn)	((pfn) < num_physpages)
#endif


static __inline__ int pfn_to_nid(unsigned long pfn)
{
	int node;

	for (node = 0 ; node < MAX_NUMNODES ; node++)
		if (pfn >= node_start_pfn(node) && pfn <= node_end_pfn(node))
			break;

	return node;
}

static __inline__ struct pglist_data *pfn_to_pgdat(unsigned long pfn)
{
	return(NODE_DATA(pfn_to_nid(pfn)));
}

#endif /* CONFIG_DISCONTIGMEM */
#endif /* _ASM_MMZONE_H_ */
