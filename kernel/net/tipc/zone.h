

#ifndef _TIPC_ZONE_H
#define _TIPC_ZONE_H

#include "node_subscr.h"
#include "net.h"



struct _zone {
	u32 addr;
	struct cluster *clusters[2]; /* currently limited to just 1 cluster */
	u32 links;
};

struct tipc_node *tipc_zone_select_remote_node(struct _zone *z_ptr, u32 addr, u32 ref);
u32 tipc_zone_select_router(struct _zone *z_ptr, u32 addr, u32 ref);
void tipc_zone_remove_as_router(struct _zone *z_ptr, u32 router);
void tipc_zone_send_external_routes(struct _zone *z_ptr, u32 dest);
struct _zone *tipc_zone_create(u32 addr);
void tipc_zone_delete(struct _zone *z_ptr);
void tipc_zone_attach_cluster(struct _zone *z_ptr, struct cluster *c_ptr);
u32 tipc_zone_next_node(u32 addr);

static inline struct _zone *tipc_zone_find(u32 addr)
{
	return tipc_net.zones[tipc_zone(addr)];
}

#endif
