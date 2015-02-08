

#ifndef _NET_TIPC_PORT_H_
#define _NET_TIPC_PORT_H_

#ifdef __KERNEL__

#include <linux/tipc.h>
#include <linux/skbuff.h>
#include <net/tipc/tipc_msg.h>

#define TIPC_FLOW_CONTROL_WIN 512


struct tipc_port {
        void *usr_handle;
        spinlock_t *lock;
	int connected;
        u32 conn_type;
        u32 conn_instance;
	u32 conn_unacked;
	int published;
	u32 congested;
	u32 max_pkt;
	u32 ref;
	struct tipc_msg phdr;
};


struct tipc_port *tipc_createport_raw(void *usr_handle,
			u32 (*dispatcher)(struct tipc_port *, struct sk_buff *),
			void (*wakeup)(struct tipc_port *),
			const u32 importance);

int tipc_reject_msg(struct sk_buff *buf, u32 err);

int tipc_send_buf_fast(struct sk_buff *buf, u32 destnode);

void tipc_acknowledge(u32 port_ref,u32 ack);

struct tipc_port *tipc_get_port(const u32 ref);

void *tipc_get_handle(const u32 ref);


int tipc_disconnect_port(struct tipc_port *tp_ptr);


#endif

#endif

