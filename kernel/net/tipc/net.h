

#ifndef _TIPC_NET_H
#define _TIPC_NET_H

struct _zone;


struct network {
	struct _zone **zones;
};


extern struct network tipc_net;
extern rwlock_t tipc_net_lock;

void tipc_net_remove_as_router(u32 router);
void tipc_net_send_external_routes(u32 dest);
void tipc_net_route_msg(struct sk_buff *buf);
struct tipc_node *tipc_net_select_remote_node(u32 addr, u32 ref);
u32 tipc_net_select_router(u32 addr, u32 ref);

int tipc_net_start(u32 addr);
void tipc_net_stop(void);

#endif
