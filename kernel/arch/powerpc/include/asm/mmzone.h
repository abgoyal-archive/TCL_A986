
#ifndef _ASM_MMZONE_H_
#define _ASM_MMZONE_H_
#ifdef __KERNEL__

#include <linux/cpumask.h>


#ifdef CONFIG_NEED_MULTIPLE_NODES

extern struct pglist_data *node_data[];
#define NODE_DATA(nid)		(node_data[nid])


extern int numa_cpu_lookup_table[];
extern cpumask_var_t node_to_cpumask_map[];
#ifdef CONFIG_MEMORY_HOTPLUG
extern unsigned long max_pfn;
#endif


#define node_start_pfn(nid)	(NODE_DATA(nid)->node_start_pfn)
#define node_end_pfn(nid)	(NODE_DATA(nid)->node_end_pfn)

#endif /* CONFIG_NEED_MULTIPLE_NODES */

#endif /* __KERNEL__ */
#endif /* _ASM_MMZONE_H_ */
