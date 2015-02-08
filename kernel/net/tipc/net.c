

#include "core.h"
#include "bearer.h"
#include "net.h"
#include "zone.h"
#include "addr.h"
#include "name_table.h"
#include "name_distr.h"
#include "subscr.h"
#include "link.h"
#include "msg.h"
#include "port.h"
#include "bcast.h"
#include "discover.h"
#include "config.h"


DEFINE_RWLOCK(tipc_net_lock);
static struct _zone *tipc_zones[256] = { NULL, };
struct network tipc_net = { tipc_zones };

struct tipc_node *tipc_net_select_remote_node(u32 addr, u32 ref)
{
	return tipc_zone_select_remote_node(tipc_net.zones[tipc_zone(addr)], addr, ref);
}

u32 tipc_net_select_router(u32 addr, u32 ref)
{
	return tipc_zone_select_router(tipc_net.zones[tipc_zone(addr)], addr, ref);
}

#if 0
u32 tipc_net_next_node(u32 a)
{
	if (tipc_net.zones[tipc_zone(a)])
		return tipc_zone_next_node(a);
	return 0;
}
#endif

void tipc_net_remove_as_router(u32 router)
{
	u32 z_num;

	for (z_num = 1; z_num <= tipc_max_zones; z_num++) {
		if (!tipc_net.zones[z_num])
			continue;
		tipc_zone_remove_as_router(tipc_net.zones[z_num], router);
	}
}

void tipc_net_send_external_routes(u32 dest)
{
	u32 z_num;

	for (z_num = 1; z_num <= tipc_max_zones; z_num++) {
		if (tipc_net.zones[z_num])
			tipc_zone_send_external_routes(tipc_net.zones[z_num], dest);
	}
}

static void net_stop(void)
{
	u32 z_num;

	for (z_num = 1; z_num <= tipc_max_zones; z_num++)
		tipc_zone_delete(tipc_net.zones[z_num]);
}

static void net_route_named_msg(struct sk_buff *buf)
{
	struct tipc_msg *msg = buf_msg(buf);
	u32 dnode;
	u32 dport;

	if (!msg_named(msg)) {
		msg_dbg(msg, "tipc_net->drop_nam:");
		buf_discard(buf);
		return;
	}

	dnode = addr_domain(msg_lookup_scope(msg));
	dport = tipc_nametbl_translate(msg_nametype(msg), msg_nameinst(msg), &dnode);
	dbg("tipc_net->lookup<%u,%u>-><%u,%x>\n",
	    msg_nametype(msg), msg_nameinst(msg), dport, dnode);
	if (dport) {
		msg_set_destnode(msg, dnode);
		msg_set_destport(msg, dport);
		tipc_net_route_msg(buf);
		return;
	}
	msg_dbg(msg, "tipc_net->rej:NO NAME: ");
	tipc_reject_msg(buf, TIPC_ERR_NO_NAME);
}

void tipc_net_route_msg(struct sk_buff *buf)
{
	struct tipc_msg *msg;
	u32 dnode;

	if (!buf)
		return;
	msg = buf_msg(buf);

	msg_incr_reroute_cnt(msg);
	if (msg_reroute_cnt(msg) > 6) {
		if (msg_errcode(msg)) {
			msg_dbg(msg, "NET>DISC>:");
			buf_discard(buf);
		} else {
			msg_dbg(msg, "NET>REJ>:");
			tipc_reject_msg(buf, msg_destport(msg) ?
					TIPC_ERR_NO_PORT : TIPC_ERR_NO_NAME);
		}
		return;
	}

	msg_dbg(msg, "tipc_net->rout: ");

	/* Handle message for this node */
	dnode = msg_short(msg) ? tipc_own_addr : msg_destnode(msg);
	if (tipc_in_scope(dnode, tipc_own_addr)) {
		if (msg_isdata(msg)) {
			if (msg_mcast(msg))
				tipc_port_recv_mcast(buf, NULL);
			else if (msg_destport(msg))
				tipc_port_recv_msg(buf);
			else
				net_route_named_msg(buf);
			return;
		}
		switch (msg_user(msg)) {
		case ROUTE_DISTRIBUTOR:
			tipc_cltr_recv_routing_table(buf);
			break;
		case NAME_DISTRIBUTOR:
			tipc_named_recv(buf);
			break;
		case CONN_MANAGER:
			tipc_port_recv_proto_msg(buf);
			break;
		default:
			msg_dbg(msg,"DROP/NET/<REC<");
			buf_discard(buf);
		}
		return;
	}

	/* Handle message for another node */
	msg_dbg(msg, "NET>SEND>: ");
	tipc_link_send(buf, dnode, msg_link_selector(msg));
}

int tipc_net_start(u32 addr)
{
	char addr_string[16];
	int res;

	if (tipc_mode != TIPC_NODE_MODE)
		return -ENOPROTOOPT;

	tipc_subscr_stop();
	tipc_cfg_stop();

	tipc_own_addr = addr;
	tipc_mode = TIPC_NET_MODE;
	tipc_named_reinit();
	tipc_port_reinit();

	if ((res = tipc_cltr_init()) ||
	    (res = tipc_bclink_init())) {
		return res;
	}

	tipc_k_signal((Handler)tipc_subscr_start, 0);
	tipc_k_signal((Handler)tipc_cfg_init, 0);

	info("Started in network mode\n");
	info("Own node address %s, network identity %u\n",
	     tipc_addr_string_fill(addr_string, tipc_own_addr), tipc_net_id);
	return 0;
}

void tipc_net_stop(void)
{
	if (tipc_mode != TIPC_NET_MODE)
		return;
	write_lock_bh(&tipc_net_lock);
	tipc_bearer_stop();
	tipc_mode = TIPC_NODE_MODE;
	tipc_bclink_stop();
	net_stop();
	write_unlock_bh(&tipc_net_lock);
	info("Left network mode\n");
}

