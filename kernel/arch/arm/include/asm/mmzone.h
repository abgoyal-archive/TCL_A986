
#ifndef __ASM_MMZONE_H
#define __ASM_MMZONE_H

extern pg_data_t discontig_node_data[];

#define NODE_DATA(nid)		(&discontig_node_data[nid])

#define NODE_MEM_MAP(nid)	(NODE_DATA(nid)->node_mem_map)

#include <mach/memory.h>

#endif
