
#ifndef _DCCP_CCID2_H_
#define _DCCP_CCID2_H_

#include <linux/dccp.h>
#include <linux/timer.h>
#include <linux/types.h>
#include "../ccid.h"
/* NUMDUPACK parameter from RFC 4341, p. 6 */
#define NUMDUPACK	3

struct sock;

struct ccid2_seq {
	u64			ccid2s_seq;
	unsigned long		ccid2s_sent;
	int			ccid2s_acked;
	struct ccid2_seq	*ccid2s_prev;
	struct ccid2_seq	*ccid2s_next;
};

#define CCID2_SEQBUF_LEN 1024
#define CCID2_SEQBUF_MAX 128

struct ccid2_hc_tx_sock {
	u32			tx_cwnd;
	u32			tx_ssthresh;
	u32			tx_pipe;
	u32			tx_packets_acked;
	struct ccid2_seq	*tx_seqbuf[CCID2_SEQBUF_MAX];
	int			tx_seqbufc;
	struct ccid2_seq	*tx_seqh;
	struct ccid2_seq	*tx_seqt;
	long			tx_rto;
	long			tx_srtt;
	long			tx_rttvar;
	unsigned long		tx_lastrtt;
	struct timer_list	tx_rtotimer;
	u64			tx_rpseq;
	int			tx_rpdupack;
	unsigned long		tx_last_cong;
	u64			tx_high_ack;
};

struct ccid2_hc_rx_sock {
	int	rx_data;
};

static inline struct ccid2_hc_tx_sock *ccid2_hc_tx_sk(const struct sock *sk)
{
	return ccid_priv(dccp_sk(sk)->dccps_hc_tx_ccid);
}

static inline struct ccid2_hc_rx_sock *ccid2_hc_rx_sk(const struct sock *sk)
{
	return ccid_priv(dccp_sk(sk)->dccps_hc_rx_ccid);
}
#endif /* _DCCP_CCID2_H_ */
