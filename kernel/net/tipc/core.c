

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/random.h>

#include "core.h"
#include "dbg.h"
#include "ref.h"
#include "net.h"
#include "user_reg.h"
#include "name_table.h"
#include "subscr.h"
#include "config.h"


#ifndef CONFIG_TIPC_ZONES
#define CONFIG_TIPC_ZONES 3
#endif

#ifndef CONFIG_TIPC_CLUSTERS
#define CONFIG_TIPC_CLUSTERS 1
#endif

#ifndef CONFIG_TIPC_NODES
#define CONFIG_TIPC_NODES 255
#endif

#ifndef CONFIG_TIPC_SLAVE_NODES
#define CONFIG_TIPC_SLAVE_NODES 0
#endif

#ifndef CONFIG_TIPC_PORTS
#define CONFIG_TIPC_PORTS 8191
#endif

#ifndef CONFIG_TIPC_LOG
#define CONFIG_TIPC_LOG 0
#endif

/* global variables used by multiple sub-systems within TIPC */

int tipc_mode = TIPC_NOT_RUNNING;
int tipc_random;
atomic_t tipc_user_count = ATOMIC_INIT(0);

const char tipc_alphabet[] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_.";

/* configurable TIPC parameters */

u32 tipc_own_addr;
int tipc_max_zones;
int tipc_max_clusters;
int tipc_max_nodes;
int tipc_max_slaves;
int tipc_max_ports;
int tipc_max_subscriptions;
int tipc_max_publications;
int tipc_net_id;
int tipc_remote_management;


int tipc_get_mode(void)
{
	return tipc_mode;
}


struct sk_buff *buf_acquire(u32 size)
{
	struct sk_buff *skb;
	unsigned int buf_size = (BUF_HEADROOM + size + 3) & ~3u;

	skb = alloc_skb_fclone(buf_size, GFP_ATOMIC);
	if (skb) {
		skb_reserve(skb, BUF_HEADROOM);
		skb_put(skb, size);
		skb->next = NULL;
	}
	return skb;
}


void tipc_core_stop_net(void)
{
	tipc_eth_media_stop();
	tipc_net_stop();
}


int tipc_core_start_net(unsigned long addr)
{
	int res;

	if ((res = tipc_net_start(addr)) ||
	    (res = tipc_eth_media_start())) {
		tipc_core_stop_net();
	}
	return res;
}


void tipc_core_stop(void)
{
	if (tipc_mode != TIPC_NODE_MODE)
		return;

	tipc_mode = TIPC_NOT_RUNNING;

	tipc_netlink_stop();
	tipc_handler_stop();
	tipc_cfg_stop();
	tipc_subscr_stop();
	tipc_reg_stop();
	tipc_nametbl_stop();
	tipc_ref_table_stop();
	tipc_socket_stop();
}


int tipc_core_start(void)
{
	int res;

	if (tipc_mode != TIPC_NOT_RUNNING)
		return -ENOPROTOOPT;

	get_random_bytes(&tipc_random, sizeof(tipc_random));
	tipc_mode = TIPC_NODE_MODE;

	if ((res = tipc_handler_start()) ||
	    (res = tipc_ref_table_init(tipc_max_ports, tipc_random)) ||
	    (res = tipc_reg_start()) ||
	    (res = tipc_nametbl_init()) ||
	    (res = tipc_k_signal((Handler)tipc_subscr_start, 0)) ||
	    (res = tipc_k_signal((Handler)tipc_cfg_init, 0)) ||
	    (res = tipc_netlink_start()) ||
	    (res = tipc_socket_init())) {
		tipc_core_stop();
	}
	return res;
}


static int __init tipc_init(void)
{
	int res;

	tipc_log_resize(CONFIG_TIPC_LOG);
	info("Activated (version " TIPC_MOD_VER
	     " compiled " __DATE__ " " __TIME__ ")\n");

	tipc_own_addr = 0;
	tipc_remote_management = 1;
	tipc_max_publications = 10000;
	tipc_max_subscriptions = 2000;
	tipc_max_ports = CONFIG_TIPC_PORTS;
	tipc_max_zones = CONFIG_TIPC_ZONES;
	tipc_max_clusters = CONFIG_TIPC_CLUSTERS;
	tipc_max_nodes = CONFIG_TIPC_NODES;
	tipc_max_slaves = CONFIG_TIPC_SLAVE_NODES;
	tipc_net_id = 4711;

	if ((res = tipc_core_start()))
		err("Unable to start in single node mode\n");
	else
		info("Started in single node mode\n");
	return res;
}

static void __exit tipc_exit(void)
{
	tipc_core_stop_net();
	tipc_core_stop();
	info("Deactivated\n");
	tipc_log_resize(0);
}

module_init(tipc_init);
module_exit(tipc_exit);

MODULE_DESCRIPTION("TIPC: Transparent Inter Process Communication");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(TIPC_MOD_VER);

/* Native TIPC API for kernel-space applications (see tipc.h) */

EXPORT_SYMBOL(tipc_attach);
EXPORT_SYMBOL(tipc_detach);
EXPORT_SYMBOL(tipc_get_addr);
EXPORT_SYMBOL(tipc_get_mode);
EXPORT_SYMBOL(tipc_createport);
EXPORT_SYMBOL(tipc_deleteport);
EXPORT_SYMBOL(tipc_ownidentity);
EXPORT_SYMBOL(tipc_portimportance);
EXPORT_SYMBOL(tipc_set_portimportance);
EXPORT_SYMBOL(tipc_portunreliable);
EXPORT_SYMBOL(tipc_set_portunreliable);
EXPORT_SYMBOL(tipc_portunreturnable);
EXPORT_SYMBOL(tipc_set_portunreturnable);
EXPORT_SYMBOL(tipc_publish);
EXPORT_SYMBOL(tipc_withdraw);
EXPORT_SYMBOL(tipc_connect2port);
EXPORT_SYMBOL(tipc_disconnect);
EXPORT_SYMBOL(tipc_shutdown);
EXPORT_SYMBOL(tipc_isconnected);
EXPORT_SYMBOL(tipc_peer);
EXPORT_SYMBOL(tipc_ref_valid);
EXPORT_SYMBOL(tipc_send);
EXPORT_SYMBOL(tipc_send_buf);
EXPORT_SYMBOL(tipc_send2name);
EXPORT_SYMBOL(tipc_forward2name);
EXPORT_SYMBOL(tipc_send_buf2name);
EXPORT_SYMBOL(tipc_forward_buf2name);
EXPORT_SYMBOL(tipc_send2port);
EXPORT_SYMBOL(tipc_forward2port);
EXPORT_SYMBOL(tipc_send_buf2port);
EXPORT_SYMBOL(tipc_forward_buf2port);
EXPORT_SYMBOL(tipc_multicast);
/* EXPORT_SYMBOL(tipc_multicast_buf); not available yet */
EXPORT_SYMBOL(tipc_ispublished);
EXPORT_SYMBOL(tipc_available_nodes);

/* TIPC API for external bearers (see tipc_bearer.h) */

EXPORT_SYMBOL(tipc_block_bearer);
EXPORT_SYMBOL(tipc_continue);
EXPORT_SYMBOL(tipc_disable_bearer);
EXPORT_SYMBOL(tipc_enable_bearer);
EXPORT_SYMBOL(tipc_recv_msg);
EXPORT_SYMBOL(tipc_register_media);

/* TIPC API for external APIs (see tipc_port.h) */

EXPORT_SYMBOL(tipc_createport_raw);
EXPORT_SYMBOL(tipc_reject_msg);
EXPORT_SYMBOL(tipc_send_buf_fast);
EXPORT_SYMBOL(tipc_acknowledge);
EXPORT_SYMBOL(tipc_get_port);
EXPORT_SYMBOL(tipc_get_handle);

