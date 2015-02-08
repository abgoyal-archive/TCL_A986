

#ifndef _DCCP_PKT_HIST_
#define _DCCP_PKT_HIST_

#include <linux/list.h>
#include <linux/slab.h>
#include "tfrc.h"

struct tfrc_tx_hist_entry;

extern int  tfrc_tx_hist_add(struct tfrc_tx_hist_entry **headp, u64 seqno);
extern void tfrc_tx_hist_purge(struct tfrc_tx_hist_entry **headp);
extern u32  tfrc_tx_hist_rtt(struct tfrc_tx_hist_entry *head,
			     const u64 seqno, const ktime_t now);

/* Subtraction a-b modulo-16, respects circular wrap-around */
#define SUB16(a, b) (((a) + 16 - (b)) & 0xF)

/* Number of packets to wait after a missing packet (RFC 4342, 6.1) */
#define TFRC_NDUPACK 3

struct tfrc_rx_hist_entry {
	u64		 tfrchrx_seqno:48,
			 tfrchrx_ccval:4,
			 tfrchrx_type:4;
	u64		 tfrchrx_ndp:48;
	ktime_t		 tfrchrx_tstamp;
};

struct tfrc_rx_hist {
	struct tfrc_rx_hist_entry *ring[TFRC_NDUPACK + 1];
	u8			  loss_count:2,
				  loss_start:2;
#define rtt_sample_prev		  loss_start
};

static inline u8 tfrc_rx_hist_index(const struct tfrc_rx_hist *h, const u8 n)
{
	return (h->loss_start + n) & TFRC_NDUPACK;
}

static inline struct tfrc_rx_hist_entry *
			tfrc_rx_hist_last_rcv(const struct tfrc_rx_hist *h)
{
	return h->ring[tfrc_rx_hist_index(h, h->loss_count)];
}

static inline struct tfrc_rx_hist_entry *
			tfrc_rx_hist_entry(const struct tfrc_rx_hist *h, const u8 n)
{
	return h->ring[tfrc_rx_hist_index(h, n)];
}

static inline struct tfrc_rx_hist_entry *
			tfrc_rx_hist_loss_prev(const struct tfrc_rx_hist *h)
{
	return h->ring[h->loss_start];
}

/* indicate whether previously a packet was detected missing */
static inline bool tfrc_rx_hist_loss_pending(const struct tfrc_rx_hist *h)
{
	return h->loss_count > 0;
}

extern void tfrc_rx_hist_add_packet(struct tfrc_rx_hist *h,
				    const struct sk_buff *skb, const u64 ndp);

extern int tfrc_rx_hist_duplicate(struct tfrc_rx_hist *h, struct sk_buff *skb);

struct tfrc_loss_hist;
extern int  tfrc_rx_handle_loss(struct tfrc_rx_hist *h,
				struct tfrc_loss_hist *lh,
				struct sk_buff *skb, const u64 ndp,
				u32 (*first_li)(struct sock *sk),
				struct sock *sk);
extern u32 tfrc_rx_hist_sample_rtt(struct tfrc_rx_hist *h,
				   const struct sk_buff *skb);
extern int tfrc_rx_hist_alloc(struct tfrc_rx_hist *h);
extern void tfrc_rx_hist_purge(struct tfrc_rx_hist *h);

#endif /* _DCCP_PKT_HIST_ */
