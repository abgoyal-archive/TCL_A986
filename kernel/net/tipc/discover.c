

#include "core.h"
#include "dbg.h"
#include "link.h"
#include "zone.h"
#include "discover.h"
#include "port.h"
#include "name_table.h"

#define TIPC_LINK_REQ_INIT	125	/* min delay during bearer start up */
#define TIPC_LINK_REQ_FAST	2000	/* normal delay if bearer has no links */
#define TIPC_LINK_REQ_SLOW	600000	/* normal delay if bearer has links */

#if 0
#define  GET_NODE_INFO         300
#define  GET_NODE_INFO_RESULT  301
#define  FORWARD_LINK_PROBE    302
#define  LINK_REQUEST_REJECTED 303
#define  LINK_REQUEST_ACCEPTED 304
#define  DROP_LINK_REQUEST     305
#define  CHECK_LINK_COUNT      306
#endif



struct link_req {
	struct bearer *bearer;
	struct tipc_media_addr dest;
	struct sk_buff *buf;
	struct timer_list timer;
	unsigned int timer_intv;
};


#if 0
int disc_create_link(const struct tipc_link_create *argv)
{
	/*
	 * Code for inter cluster link setup here
	 */
	return TIPC_OK;
}
#endif


void tipc_disc_link_event(u32 addr, char *name, int up)
{
	if (in_own_cluster(addr))
		return;
	/*
	 * Code for inter cluster link setup here
	 */
}


static struct sk_buff *tipc_disc_init_msg(u32 type,
					  u32 req_links,
					  u32 dest_domain,
					  struct bearer *b_ptr)
{
	struct sk_buff *buf = buf_acquire(DSC_H_SIZE);
	struct tipc_msg *msg;

	if (buf) {
		msg = buf_msg(buf);
		tipc_msg_init(msg, LINK_CONFIG, type, DSC_H_SIZE, dest_domain);
		msg_set_non_seq(msg, 1);
		msg_set_req_links(msg, req_links);
		msg_set_dest_domain(msg, dest_domain);
		msg_set_bc_netid(msg, tipc_net_id);
		msg_set_media_addr(msg, &b_ptr->publ.addr);
	}
	return buf;
}


static void disc_dupl_alert(struct bearer *b_ptr, u32 node_addr,
			    struct tipc_media_addr *media_addr)
{
	char node_addr_str[16];
	char media_addr_str[64];
	struct print_buf pb;

	tipc_addr_string_fill(node_addr_str, node_addr);
	tipc_printbuf_init(&pb, media_addr_str, sizeof(media_addr_str));
	tipc_media_addr_printf(&pb, media_addr);
	tipc_printbuf_validate(&pb);
	warn("Duplicate %s using %s seen on <%s>\n",
	     node_addr_str, media_addr_str, b_ptr->publ.name);
}


void tipc_disc_recv_msg(struct sk_buff *buf, struct bearer *b_ptr)
{
	struct link *link;
	struct tipc_media_addr media_addr;
	struct tipc_msg *msg = buf_msg(buf);
	u32 dest = msg_dest_domain(msg);
	u32 orig = msg_prevnode(msg);
	u32 net_id = msg_bc_netid(msg);
	u32 type = msg_type(msg);

	msg_get_media_addr(msg,&media_addr);
	msg_dbg(msg, "RECV:");
	buf_discard(buf);

	if (net_id != tipc_net_id)
		return;
	if (!tipc_addr_domain_valid(dest))
		return;
	if (!tipc_addr_node_valid(orig))
		return;
	if (orig == tipc_own_addr) {
		if (memcmp(&media_addr, &b_ptr->publ.addr, sizeof(media_addr)))
			disc_dupl_alert(b_ptr, tipc_own_addr, &media_addr);
		return;
	}
	if (!tipc_in_scope(dest, tipc_own_addr))
		return;
	if (is_slave(tipc_own_addr) && is_slave(orig))
		return;
	if (is_slave(orig) && !in_own_cluster(orig))
		return;
	if (in_own_cluster(orig)) {
		/* Always accept link here */
		struct sk_buff *rbuf;
		struct tipc_media_addr *addr;
		struct tipc_node *n_ptr = tipc_node_find(orig);
		int link_fully_up;

		dbg(" in own cluster\n");
		if (n_ptr == NULL) {
			n_ptr = tipc_node_create(orig);
			if (!n_ptr)
				return;
		}
		spin_lock_bh(&n_ptr->lock);
		link = n_ptr->links[b_ptr->identity];
		if (!link) {
			dbg("creating link\n");
			link = tipc_link_create(b_ptr, orig, &media_addr);
			if (!link) {
				spin_unlock_bh(&n_ptr->lock);
				return;
			}
		}
		addr = &link->media_addr;
		if (memcmp(addr, &media_addr, sizeof(*addr))) {
			if (tipc_link_is_up(link) || (!link->started)) {
				disc_dupl_alert(b_ptr, orig, &media_addr);
				spin_unlock_bh(&n_ptr->lock);
				return;
			}
			warn("Resetting link <%s>, peer interface address changed\n",
			     link->name);
			memcpy(addr, &media_addr, sizeof(*addr));
			tipc_link_reset(link);
		}
		link_fully_up = link_working_working(link);
		spin_unlock_bh(&n_ptr->lock);
		if ((type == DSC_RESP_MSG) || link_fully_up)
			return;
		rbuf = tipc_disc_init_msg(DSC_RESP_MSG, 1, orig, b_ptr);
		if (rbuf != NULL) {
			msg_dbg(buf_msg(rbuf),"SEND:");
			b_ptr->media->send_msg(rbuf, &b_ptr->publ, &media_addr);
			buf_discard(rbuf);
		}
	}
}


void tipc_disc_stop_link_req(struct link_req *req)
{
	if (!req)
		return;

	k_cancel_timer(&req->timer);
	k_term_timer(&req->timer);
	buf_discard(req->buf);
	kfree(req);
}


void tipc_disc_update_link_req(struct link_req *req)
{
	if (!req)
		return;

	if (req->timer_intv == TIPC_LINK_REQ_SLOW) {
		if (!req->bearer->nodes.count) {
			req->timer_intv = TIPC_LINK_REQ_FAST;
			k_start_timer(&req->timer, req->timer_intv);
		}
	} else if (req->timer_intv == TIPC_LINK_REQ_FAST) {
		if (req->bearer->nodes.count) {
			req->timer_intv = TIPC_LINK_REQ_SLOW;
			k_start_timer(&req->timer, req->timer_intv);
		}
	} else {
		/* leave timer "as is" if haven't yet reached a "normal" rate */
	}
}


static void disc_timeout(struct link_req *req)
{
	spin_lock_bh(&req->bearer->publ.lock);

	req->bearer->media->send_msg(req->buf, &req->bearer->publ, &req->dest);

	if ((req->timer_intv == TIPC_LINK_REQ_SLOW) ||
	    (req->timer_intv == TIPC_LINK_REQ_FAST)) {
		/* leave timer interval "as is" if already at a "normal" rate */
	} else {
		req->timer_intv *= 2;
		if (req->timer_intv > TIPC_LINK_REQ_FAST)
			req->timer_intv = TIPC_LINK_REQ_FAST;
		if ((req->timer_intv == TIPC_LINK_REQ_FAST) &&
		    (req->bearer->nodes.count))
			req->timer_intv = TIPC_LINK_REQ_SLOW;
	}
	k_start_timer(&req->timer, req->timer_intv);

	spin_unlock_bh(&req->bearer->publ.lock);
}


struct link_req *tipc_disc_init_link_req(struct bearer *b_ptr,
					 const struct tipc_media_addr *dest,
					 u32 dest_domain,
					 u32 req_links)
{
	struct link_req *req;

	req = kmalloc(sizeof(*req), GFP_ATOMIC);
	if (!req)
		return NULL;

	req->buf = tipc_disc_init_msg(DSC_REQ_MSG, req_links, dest_domain, b_ptr);
	if (!req->buf) {
		kfree(req);
		return NULL;
	}

	memcpy(&req->dest, dest, sizeof(*dest));
	req->bearer = b_ptr;
	req->timer_intv = TIPC_LINK_REQ_INIT;
	k_init_timer(&req->timer, (Handler)disc_timeout, (unsigned long)req);
	k_start_timer(&req->timer, req->timer_intv);
	return req;
}

