

#ifndef IRTTP_H
#define IRTTP_H

#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>

#include <net/irda/irda.h>
#include <net/irda/irlmp.h>		/* struct lsap_cb */
#include <net/irda/qos.h>		/* struct qos_info */
#include <net/irda/irqueue.h>

#define TTP_MAX_CONNECTIONS    LM_MAX_CONNECTIONS
#define TTP_HEADER             1
#define TTP_MAX_HEADER         (TTP_HEADER + LMP_MAX_HEADER)
#define TTP_SAR_HEADER         5
#define TTP_PARAMETERS         0x80
#define TTP_MORE               0x80

/* Transmission queue sizes */
/* Worst case scenario, two window of data - Jean II */
#define TTP_TX_MAX_QUEUE	14
#define TTP_TX_LOW_THRESHOLD	5
#define TTP_TX_HIGH_THRESHOLD	7

/* Receive queue sizes */
#define TTP_RX_MIN_CREDIT	8
#define TTP_RX_DEFAULT_CREDIT	16
#define TTP_RX_MAX_CREDIT	21

/* What clients should use when calling ttp_open_tsap() */
#define DEFAULT_INITIAL_CREDIT	TTP_RX_DEFAULT_CREDIT

/* Some priorities for disconnect requests */
#define P_NORMAL    0
#define P_HIGH      1

#define TTP_SAR_DISABLE 0
#define TTP_SAR_UNBOUND 0xffffffff

/* Parameters */
#define TTP_MAX_SDU_SIZE 0x01

struct tsap_cb {
	irda_queue_t q;            /* Must be first */
	magic_t magic;        /* Just in case */

	__u8 stsap_sel;       /* Source TSAP */
	__u8 dtsap_sel;       /* Destination TSAP */

	struct lsap_cb *lsap; /* Corresponding LSAP to this TSAP */

	__u8 connected;       /* TSAP connected */
	 
	__u8 initial_credit;  /* Initial credit to give peer */

        int avail_credit;    /* Available credit to return to peer */
	int remote_credit;   /* Credit held by peer TTP entity */
	int send_credit;     /* Credit held by local TTP entity */
	
	struct sk_buff_head tx_queue; /* Frames to be transmitted */
	struct sk_buff_head rx_queue; /* Received frames */
	struct sk_buff_head rx_fragments;
	int tx_queue_lock;
	int rx_queue_lock;
	spinlock_t lock;

	notify_t notify;       /* Callbacks to client layer */

	struct net_device_stats stats;
	struct timer_list todo_timer; 

	__u32 max_seg_size;     /* Max data that fit into an IrLAP frame */
	__u8  max_header_size;

	int   rx_sdu_busy;     /* RxSdu.busy */
	__u32 rx_sdu_size;     /* Current size of a partially received frame */
	__u32 rx_max_sdu_size; /* Max receive user data size */

	int tx_sdu_busy;       /* TxSdu.busy */
	__u32 tx_max_sdu_size; /* Max transmit user data size */

	int close_pend;        /* Close, but disconnect_pend */
	unsigned long disconnect_pend; /* Disconnect, but still data to send */
	struct sk_buff *disconnect_skb;
};

struct irttp_cb {
	magic_t    magic;	
	hashbin_t *tsaps;
};

int  irttp_init(void);
void irttp_cleanup(void);

struct tsap_cb *irttp_open_tsap(__u8 stsap_sel, int credit, notify_t *notify);
int irttp_close_tsap(struct tsap_cb *self);

int irttp_data_request(struct tsap_cb *self, struct sk_buff *skb);
int irttp_udata_request(struct tsap_cb *self, struct sk_buff *skb);

int irttp_connect_request(struct tsap_cb *self, __u8 dtsap_sel, 
			  __u32 saddr, __u32 daddr,
			  struct qos_info *qos, __u32 max_sdu_size, 
			  struct sk_buff *userdata);
int irttp_connect_response(struct tsap_cb *self, __u32 max_sdu_size, 
			    struct sk_buff *userdata);
int irttp_disconnect_request(struct tsap_cb *self, struct sk_buff *skb,
			     int priority);
void irttp_flow_request(struct tsap_cb *self, LOCAL_FLOW flow);
struct tsap_cb *irttp_dup(struct tsap_cb *self, void *instance);

static inline __u32 irttp_get_saddr(struct tsap_cb *self)
{
	return irlmp_get_saddr(self->lsap);
}

static inline __u32 irttp_get_daddr(struct tsap_cb *self)
{
	return irlmp_get_daddr(self->lsap);
}

static inline __u32 irttp_get_max_seg_size(struct tsap_cb *self)
{
	return self->max_seg_size;
}

static inline void irttp_listen(struct tsap_cb *self)
{
	irlmp_listen(self->lsap);
	self->dtsap_sel = LSAP_ANY;
}

static inline int irttp_is_primary(struct tsap_cb *self)
{
	if ((self == NULL) ||
	    (self->lsap == NULL) ||
	    (self->lsap->lap == NULL) ||
	    (self->lsap->lap->irlap == NULL))
		return -2;
	return(irlap_is_primary(self->lsap->lap->irlap));
}

#endif /* IRTTP_H */
