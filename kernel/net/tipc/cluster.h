

#ifndef _TIPC_CLUSTER_H
#define _TIPC_CLUSTER_H

#include "addr.h"
#include "zone.h"

#define LOWEST_SLAVE  2048u


struct cluster {
	u32 addr;
	struct _zone *owner;
	struct tipc_node **nodes;
	u32 highest_node;
	u32 highest_slave;
};


extern struct tipc_node **tipc_local_nodes;
extern u32 tipc_highest_allowed_slave;
extern struct tipc_node_map tipc_cltr_bcast_nodes;

void tipc_cltr_remove_as_router(struct cluster *c_ptr, u32 router);
void tipc_cltr_send_ext_routes(struct cluster *c_ptr, u32 dest);
struct tipc_node *tipc_cltr_select_node(struct cluster *c_ptr, u32 selector);
u32 tipc_cltr_select_router(struct cluster *c_ptr, u32 ref);
void tipc_cltr_recv_routing_table(struct sk_buff *buf);
struct cluster *tipc_cltr_create(u32 addr);
void tipc_cltr_delete(struct cluster *c_ptr);
void tipc_cltr_attach_node(struct cluster *c_ptr, struct tipc_node *n_ptr);
void tipc_cltr_send_slave_routes(struct cluster *c_ptr, u32 dest);
void tipc_cltr_broadcast(struct sk_buff *buf);
int tipc_cltr_init(void);
u32 tipc_cltr_next_node(struct cluster *c_ptr, u32 addr);
void tipc_cltr_bcast_new_route(struct cluster *c_ptr, u32 dest, u32 lo, u32 hi);
void tipc_cltr_send_local_routes(struct cluster *c_ptr, u32 dest);
void tipc_cltr_bcast_lost_route(struct cluster *c_ptr, u32 dest, u32 lo, u32 hi);

static inline struct cluster *tipc_cltr_find(u32 addr)
{
	struct _zone *z_ptr = tipc_zone_find(addr);

	if (z_ptr)
		return z_ptr->clusters[1];
	return NULL;
}

#endif
