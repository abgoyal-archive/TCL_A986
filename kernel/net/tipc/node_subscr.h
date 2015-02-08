

#ifndef _TIPC_NODE_SUBSCR_H
#define _TIPC_NODE_SUBSCR_H

#include "addr.h"

typedef void (*net_ev_handler) (void *usr_handle);


struct tipc_node_subscr {
	struct tipc_node *node;
	net_ev_handler handle_node_down;
	void *usr_handle;
	struct list_head nodesub_list;
};

void tipc_nodesub_subscribe(struct tipc_node_subscr *node_sub, u32 addr,
			    void *usr_handle, net_ev_handler handle_down);
void tipc_nodesub_unsubscribe(struct tipc_node_subscr *node_sub);

#endif
