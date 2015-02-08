

#ifndef _CXGB3I_OFFLOAD_H
#define _CXGB3I_OFFLOAD_H

#include <linux/skbuff.h>
#include <linux/in.h>

#include "common.h"
#include "adapter.h"
#include "t3cdev.h"
#include "cxgb3_offload.h"

#define cxgb3i_log_error(fmt...) printk(KERN_ERR "cxgb3i: ERR! " fmt)
#define cxgb3i_log_warn(fmt...)	 printk(KERN_WARNING "cxgb3i: WARN! " fmt)
#define cxgb3i_log_info(fmt...)  printk(KERN_INFO "cxgb3i: " fmt)
#define cxgb3i_log_debug(fmt, args...) \
	printk(KERN_INFO "cxgb3i: %s - " fmt, __func__ , ## args)

struct s3_conn {
	struct net_device *dev;
	struct t3cdev *cdev;
	unsigned long flags;
	int tid;
	int qset;
	int mss_idx;
	struct l2t_entry *l2t;
	int wr_max;
	int wr_avail;
	int wr_unacked;
	struct sk_buff *wr_pending_head;
	struct sk_buff *wr_pending_tail;
	struct sk_buff *cpl_close;
	struct sk_buff *cpl_abort_req;
	struct sk_buff *cpl_abort_rpl;
	spinlock_t lock;
	atomic_t refcnt;
	volatile unsigned int state;
	struct sockaddr_in saddr;
	struct sockaddr_in daddr;
	struct dst_entry *dst_cache;
	struct sk_buff_head receive_queue;
	struct sk_buff_head write_queue;
	struct timer_list retry_timer;
	int err;
	rwlock_t callback_lock;
	void *user_data;

	u32 rcv_nxt;
	u32 copied_seq;
	u32 rcv_wup;
	u32 snd_nxt;
	u32 snd_una;
	u32 write_seq;
};

enum conn_states {
	C3CN_STATE_CONNECTING = 1,
	C3CN_STATE_ESTABLISHED,
	C3CN_STATE_ACTIVE_CLOSE,
	C3CN_STATE_PASSIVE_CLOSE,
	C3CN_STATE_CLOSE_WAIT_1,
	C3CN_STATE_CLOSE_WAIT_2,
	C3CN_STATE_ABORTING,
	C3CN_STATE_CLOSED,
};

static inline unsigned int c3cn_is_closing(const struct s3_conn *c3cn)
{
	return c3cn->state >= C3CN_STATE_ACTIVE_CLOSE;
}
static inline unsigned int c3cn_is_established(const struct s3_conn *c3cn)
{
	return c3cn->state == C3CN_STATE_ESTABLISHED;
}

enum c3cn_flags {
	C3CN_ABORT_RPL_RCVD,	/* received one ABORT_RPL_RSS message */
	C3CN_ABORT_REQ_RCVD,	/* received one ABORT_REQ_RSS message */
	C3CN_ABORT_RPL_PENDING,	/* expecting an abort reply */
	C3CN_TX_DATA_SENT,	/* already sent a TX_DATA WR */
	C3CN_ACTIVE_CLOSE_NEEDED,	/* need to be closed */
	C3CN_OFFLOAD_DOWN	/* offload function off */
};

struct cxgb3i_sdev_data {
	struct list_head list;
	struct t3cdev *cdev;
	struct cxgb3_client *client;
	struct adap_ports ports;
	spinlock_t lock;
	unsigned int sport_next;
	struct s3_conn *sport_conn[0];
};
#define NDEV2CDATA(ndev) (*(struct cxgb3i_sdev_data **)&(ndev)->ec_ptr)
#define CXGB3_SDEV_DATA(cdev) NDEV2CDATA((cdev)->lldev)

void cxgb3i_sdev_cleanup(void);
int cxgb3i_sdev_init(cxgb3_cpl_handler_func *);
void cxgb3i_sdev_add(struct t3cdev *, struct cxgb3_client *);
void cxgb3i_sdev_remove(struct t3cdev *);

struct s3_conn *cxgb3i_c3cn_create(void);
int cxgb3i_c3cn_connect(struct net_device *, struct s3_conn *,
			struct sockaddr_in *);
void cxgb3i_c3cn_rx_credits(struct s3_conn *, int);
int cxgb3i_c3cn_send_pdus(struct s3_conn *, struct sk_buff *);
void cxgb3i_c3cn_release(struct s3_conn *);

struct cxgb3_skb_rx_cb {
	__u32 ddigest;			/* data digest */
	__u32 pdulen;			/* recovered pdu length */
};

struct cxgb3_skb_tx_cb {
	struct sk_buff *wr_next;	/* next wr */
};

struct cxgb3_skb_cb {
	__u8 flags;
	__u8 ulp_mode;
	__u32 seq;
	union {
		struct cxgb3_skb_rx_cb rx;
		struct cxgb3_skb_tx_cb tx;
	};
};

#define CXGB3_SKB_CB(skb)	((struct cxgb3_skb_cb *)&((skb)->cb[0]))
#define skb_flags(skb)		(CXGB3_SKB_CB(skb)->flags)
#define skb_ulp_mode(skb)	(CXGB3_SKB_CB(skb)->ulp_mode)
#define skb_tcp_seq(skb)	(CXGB3_SKB_CB(skb)->seq)
#define skb_rx_ddigest(skb)	(CXGB3_SKB_CB(skb)->rx.ddigest)
#define skb_rx_pdulen(skb)	(CXGB3_SKB_CB(skb)->rx.pdulen)
#define skb_tx_wr_next(skb)	(CXGB3_SKB_CB(skb)->tx.wr_next)

enum c3cb_flags {
	C3CB_FLAG_NEED_HDR = 1 << 0,	/* packet needs a TX_DATA_WR header */
	C3CB_FLAG_NO_APPEND = 1 << 1,	/* don't grow this skb */
	C3CB_FLAG_COMPL = 1 << 2,	/* request WR completion */
};

struct sge_opaque_hdr {
	void *dev;
	dma_addr_t addr[MAX_SKB_FRAGS + 1];
};

/* for TX: a skb must have a headroom of at least TX_HEADER_LEN bytes */
#define TX_HEADER_LEN \
		(sizeof(struct tx_data_wr) + sizeof(struct sge_opaque_hdr))
#define SKB_TX_HEADROOM		SKB_MAX_HEAD(TX_HEADER_LEN)

#define cxgb3i_get_private_ipv4addr(ndev) \
	(((struct port_info *)(netdev_priv(ndev)))->iscsi_ipv4addr)
#define cxgb3i_set_private_ipv4addr(ndev, addr) \
	(((struct port_info *)(netdev_priv(ndev)))->iscsi_ipv4addr) = addr

/* max. connections per adapter */
#define CXGB3I_MAX_CONN		16384
#endif /* _CXGB3_OFFLOAD_H */
