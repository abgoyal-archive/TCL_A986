
#ifndef _ASM_IA64_NODEDATA_H
#define _ASM_IA64_NODEDATA_H

#include <linux/numa.h>

#include <asm/percpu.h>
#include <asm/mmzone.h>

#ifdef CONFIG_NUMA


struct pglist_data;
struct ia64_node_data {
	short			active_cpu_count;
	short			node;
	struct pglist_data	*pg_data_ptrs[MAX_NUMNODES];
};


#define local_node_data		(local_cpu_data->node_data)

#define NODE_DATA(nid)		(local_node_data->pg_data_ptrs[nid])

#define LOCAL_DATA_ADDR(pgdat)  			\
	((struct ia64_node_data *)((u64)(pgdat) + 	\
				   L1_CACHE_ALIGN(sizeof(struct pglist_data))))

#endif /* CONFIG_NUMA */

#endif /* _ASM_IA64_NODEDATA_H */
