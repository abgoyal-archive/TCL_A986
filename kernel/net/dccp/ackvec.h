
#ifndef _ACKVEC_H
#define _ACKVEC_H

#include <linux/dccp.h>
#include <linux/compiler.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/types.h>

/* We can spread an ack vector across multiple options */
#define DCCP_MAX_ACKVEC_LEN (DCCP_SINGLE_OPT_MAXLEN * 2)

/* Estimated minimum average Ack Vector length - used for updating MPS */
#define DCCPAV_MIN_OPTLEN	16

#define DCCP_ACKVEC_STATE_RECEIVED	0
#define DCCP_ACKVEC_STATE_ECN_MARKED	(1 << 6)
#define DCCP_ACKVEC_STATE_NOT_RECEIVED	(3 << 6)

#define DCCP_ACKVEC_STATE_MASK		0xC0 /* 11000000 */
#define DCCP_ACKVEC_LEN_MASK		0x3F /* 00111111 */

struct dccp_ackvec {
	u64			av_buf_ackno;
	struct list_head	av_records;
	ktime_t			av_time;
	u16			av_buf_head;
	u16			av_vec_len;
	u8			av_buf_nonce;
	u8			av_ack_nonce;
	u8			av_buf[DCCP_MAX_ACKVEC_LEN];
};

struct dccp_ackvec_record {
	struct list_head avr_node;
	u64		 avr_ack_seqno;
	u64		 avr_ack_ackno;
	u16		 avr_ack_ptr;
	u16		 avr_sent_len;
	u8		 avr_ack_nonce;
};

struct sock;
struct sk_buff;

extern int dccp_ackvec_init(void);
extern void dccp_ackvec_exit(void);

extern struct dccp_ackvec *dccp_ackvec_alloc(const gfp_t priority);
extern void dccp_ackvec_free(struct dccp_ackvec *av);

extern int dccp_ackvec_add(struct dccp_ackvec *av, const struct sock *sk,
			   const u64 ackno, const u8 state);

extern void dccp_ackvec_check_rcv_ackno(struct dccp_ackvec *av,
					struct sock *sk, const u64 ackno);
extern int dccp_ackvec_parse(struct sock *sk, const struct sk_buff *skb,
			     u64 *ackno, const u8 opt,
			     const u8 *value, const u8 len);

extern int dccp_insert_option_ackvec(struct sock *sk, struct sk_buff *skb);

static inline int dccp_ackvec_pending(const struct dccp_ackvec *av)
{
	return av->av_vec_len;
}
#endif /* _ACKVEC_H */
