

#ifndef _TIPC_NODE_H
#define _TIPC_NODE_H

#include "node_subscr.h"
#include "addr.h"
#include "cluster.h"
#include "bearer.h"


struct tipc_node {
	u32 addr;
	spinlock_t lock;
	struct cluster *owner;
	struct tipc_node *next;
	struct list_head nsub;
	struct link *active_links[2];
	struct link *links[MAX_BEARERS];
	int link_cnt;
	int working_links;
	int permit_changeover;
	u32 routers[512/32];
	int last_router;
	struct {
		int supported;
		u32 acked;
		u32 last_in;
		u32 gap_after;
		u32 gap_to;
		u32 nack_sync;
		struct sk_buff *deferred_head;
		struct sk_buff *deferred_tail;
		struct sk_buff *defragm;
	} bclink;
};

extern struct tipc_node *tipc_nodes;
extern u32 tipc_own_tag;

struct tipc_node *tipc_node_create(u32 addr);
void tipc_node_delete(struct tipc_node *n_ptr);
struct tipc_node *tipc_node_attach_link(struct link *l_ptr);
void tipc_node_detach_link(struct tipc_node *n_ptr, struct link *l_ptr);
void tipc_node_link_down(struct tipc_node *n_ptr, struct link *l_ptr);
void tipc_node_link_up(struct tipc_node *n_ptr, struct link *l_ptr);
int tipc_node_has_active_links(struct tipc_node *n_ptr);
int tipc_node_has_redundant_links(struct tipc_node *n_ptr);
u32 tipc_node_select_router(struct tipc_node *n_ptr, u32 ref);
struct tipc_node *tipc_node_select_next_hop(u32 addr, u32 selector);
int tipc_node_is_up(struct tipc_node *n_ptr);
void tipc_node_add_router(struct tipc_node *n_ptr, u32 router);
void tipc_node_remove_router(struct tipc_node *n_ptr, u32 router);
struct sk_buff *tipc_node_get_links(const void *req_tlv_area, int req_tlv_space);
struct sk_buff *tipc_node_get_nodes(const void *req_tlv_area, int req_tlv_space);

static inline struct tipc_node *tipc_node_find(u32 addr)
{
	if (likely(in_own_cluster(addr)))
		return tipc_local_nodes[tipc_node(addr)];
	else if (tipc_addr_domain_valid(addr)) {
		struct cluster *c_ptr = tipc_cltr_find(addr);

		if (c_ptr)
			return c_ptr->nodes[tipc_node(addr)];
	}
	return NULL;
}

static inline struct tipc_node *tipc_node_select(u32 addr, u32 selector)
{
	if (likely(in_own_cluster(addr)))
		return tipc_local_nodes[tipc_node(addr)];
	return tipc_node_select_next_hop(addr, selector);
}

static inline void tipc_node_lock(struct tipc_node *n_ptr)
{
	spin_lock_bh(&n_ptr->lock);
}

static inline void tipc_node_unlock(struct tipc_node *n_ptr)
{
	spin_unlock_bh(&n_ptr->lock);
}

#endif
