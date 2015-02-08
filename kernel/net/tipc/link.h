

#ifndef _TIPC_LINK_H
#define _TIPC_LINK_H

#include "dbg.h"
#include "msg.h"
#include "bearer.h"
#include "node.h"

#define PUSH_FAILED   1
#define PUSH_FINISHED 2


#define WORKING_WORKING 560810u
#define WORKING_UNKNOWN 560811u
#define RESET_UNKNOWN   560812u
#define RESET_RESET     560813u


#define MAX_PKT_DEFAULT 1500


struct link {
	u32 addr;
	char name[TIPC_MAX_LINK_NAME];
	struct tipc_media_addr media_addr;
	struct timer_list timer;
	struct tipc_node *owner;
	struct list_head link_list;

	/* Management and link supervision data */
	int started;
	u32 checkpoint;
	u32 peer_session;
	u32 peer_bearer_id;
	struct bearer *b_ptr;
	u32 tolerance;
	u32 continuity_interval;
	u32 abort_limit;
	int state;
	int blocked;
	u32 fsm_msg_cnt;
	struct {
		unchar hdr[INT_H_SIZE];
		unchar body[TIPC_MAX_IF_NAME];
	} proto_msg;
	struct tipc_msg *pmsg;
	u32 priority;
	u32 queue_limit[15];	/* queue_limit[0]==window limit */

	/* Changeover */
	u32 exp_msg_count;
	u32 reset_checkpoint;

	/* Max packet negotiation */
	u32 max_pkt;
	u32 max_pkt_target;
	u32 max_pkt_probes;

	/* Sending */
	u32 out_queue_size;
	struct sk_buff *first_out;
	struct sk_buff *last_out;
	u32 next_out_no;
	u32 last_retransmitted;
	u32 stale_count;

	/* Reception */
	u32 next_in_no;
	u32 deferred_inqueue_sz;
	struct sk_buff *oldest_deferred_in;
	struct sk_buff *newest_deferred_in;
	u32 unacked_window;

	/* Congestion handling */
	struct sk_buff *proto_msg_queue;
	u32 retransm_queue_size;
	u32 retransm_queue_head;
	struct sk_buff *next_out;
	struct list_head waiting_ports;

	/* Fragmentation/defragmentation */
	u32 long_msg_seq_no;
	struct sk_buff *defragm_buf;

	/* Statistics */
	struct {
		u32 sent_info;		/* used in counting # sent packets */
		u32 recv_info;		/* used in counting # recv'd packets */
		u32 sent_states;
		u32 recv_states;
		u32 sent_probes;
		u32 recv_probes;
		u32 sent_nacks;
		u32 recv_nacks;
		u32 sent_acks;
		u32 sent_bundled;
		u32 sent_bundles;
		u32 recv_bundled;
		u32 recv_bundles;
		u32 retransmitted;
		u32 sent_fragmented;
		u32 sent_fragments;
		u32 recv_fragmented;
		u32 recv_fragments;
		u32 link_congs;		/* # port sends blocked by congestion */
		u32 bearer_congs;
		u32 deferred_recv;
		u32 duplicates;

		/* for statistical profiling of send queue size */

		u32 max_queue_sz;
		u32 accu_queue_sz;
		u32 queue_sz_counts;

		/* for statistical profiling of message lengths */

		u32 msg_length_counts;
		u32 msg_lengths_total;
		u32 msg_length_profile[7];
#if 0
		u32 sent_tunneled;
		u32 recv_tunneled;
#endif
	} stats;

	struct print_buf print_buf;
};

struct port;

struct link *tipc_link_create(struct bearer *b_ptr, const u32 peer,
			      const struct tipc_media_addr *media_addr);
void tipc_link_delete(struct link *l_ptr);
void tipc_link_changeover(struct link *l_ptr);
void tipc_link_send_duplicate(struct link *l_ptr, struct link *dest);
void tipc_link_reset_fragments(struct link *l_ptr);
int tipc_link_is_up(struct link *l_ptr);
int tipc_link_is_active(struct link *l_ptr);
void tipc_link_start(struct link *l_ptr);
u32 tipc_link_push_packet(struct link *l_ptr);
void tipc_link_stop(struct link *l_ptr);
struct sk_buff *tipc_link_cmd_config(const void *req_tlv_area, int req_tlv_space, u16 cmd);
struct sk_buff *tipc_link_cmd_show_stats(const void *req_tlv_area, int req_tlv_space);
struct sk_buff *tipc_link_cmd_reset_stats(const void *req_tlv_area, int req_tlv_space);
void tipc_link_reset(struct link *l_ptr);
int tipc_link_send(struct sk_buff *buf, u32 dest, u32 selector);
int tipc_link_send_buf(struct link *l_ptr, struct sk_buff *buf);
u32 tipc_link_get_max_pkt(u32 dest,u32 selector);
int tipc_link_send_sections_fast(struct port* sender,
				 struct iovec const *msg_sect,
				 const u32 num_sect,
				 u32 destnode);
int tipc_link_send_long_buf(struct link *l_ptr, struct sk_buff *buf);
void tipc_link_tunnel(struct link *l_ptr, struct tipc_msg *tnl_hdr,
		      struct tipc_msg *msg, u32 selector);
void tipc_link_recv_bundle(struct sk_buff *buf);
int  tipc_link_recv_fragment(struct sk_buff **pending,
			     struct sk_buff **fb,
			     struct tipc_msg **msg);
void tipc_link_send_proto_msg(struct link *l_ptr, u32 msg_typ, int prob, u32 gap,
			      u32 tolerance, u32 priority, u32 acked_mtu);
void tipc_link_push_queue(struct link *l_ptr);
u32 tipc_link_defer_pkt(struct sk_buff **head, struct sk_buff **tail,
		   struct sk_buff *buf);
void tipc_link_wakeup_ports(struct link *l_ptr, int all);
void tipc_link_set_queue_limits(struct link *l_ptr, u32 window);
void tipc_link_retransmit(struct link *l_ptr, struct sk_buff *start, u32 retransmits);


static inline u32 mod(u32 x)
{
	return x & 0xffffu;
}

static inline int between(u32 lower, u32 upper, u32 n)
{
	if ((lower < n) && (n < upper))
		return 1;
	if ((upper < lower) && ((n > lower) || (n < upper)))
		return 1;
	return 0;
}

static inline int less_eq(u32 left, u32 right)
{
	return (mod(right - left) < 32768u);
}

static inline int less(u32 left, u32 right)
{
	return (less_eq(left, right) && (mod(right) != mod(left)));
}

static inline u32 lesser(u32 left, u32 right)
{
	return less_eq(left, right) ? left : right;
}



static inline int link_working_working(struct link *l_ptr)
{
	return (l_ptr->state == WORKING_WORKING);
}

static inline int link_working_unknown(struct link *l_ptr)
{
	return (l_ptr->state == WORKING_UNKNOWN);
}

static inline int link_reset_unknown(struct link *l_ptr)
{
	return (l_ptr->state == RESET_UNKNOWN);
}

static inline int link_reset_reset(struct link *l_ptr)
{
	return (l_ptr->state == RESET_RESET);
}

static inline int link_blocked(struct link *l_ptr)
{
	return (l_ptr->exp_msg_count || l_ptr->blocked);
}

static inline int link_congested(struct link *l_ptr)
{
	return (l_ptr->out_queue_size >= l_ptr->queue_limit[0]);
}

#endif
