
#ifndef _DCCP_CCID3_H_
#define _DCCP_CCID3_H_

#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/types.h>
#include <linux/tfrc.h>
#include "lib/tfrc.h"
#include "../ccid.h"

/* Two seconds as per RFC 3448 4.2 */
#define TFRC_INITIAL_TIMEOUT	   (2 * USEC_PER_SEC)

/* In usecs - half the scheduling granularity as per RFC3448 4.6 */
#define TFRC_OPSYS_HALF_TIME_GRAN  (USEC_PER_SEC / (2 * HZ))

/* Parameter t_mbi from [RFC 3448, 4.3]: backoff interval in seconds */
#define TFRC_T_MBI		   64

enum ccid3_options {
	TFRC_OPT_LOSS_EVENT_RATE = 192,
	TFRC_OPT_LOSS_INTERVALS	 = 193,
	TFRC_OPT_RECEIVE_RATE	 = 194,
};

struct ccid3_options_received {
	u64 ccid3or_seqno:48,
	    ccid3or_loss_intervals_idx:16;
	u16 ccid3or_loss_intervals_len;
	u32 ccid3or_loss_event_rate;
	u32 ccid3or_receive_rate;
};

/* TFRC sender states */
enum ccid3_hc_tx_states {
	TFRC_SSTATE_NO_SENT = 1,
	TFRC_SSTATE_NO_FBACK,
	TFRC_SSTATE_FBACK,
	TFRC_SSTATE_TERM,
};

struct ccid3_hc_tx_sock {
	struct tfrc_tx_info		tx_tfrc;
#define tx_x				tx_tfrc.tfrctx_x
#define tx_x_recv			tx_tfrc.tfrctx_x_recv
#define tx_x_calc			tx_tfrc.tfrctx_x_calc
#define tx_rtt				tx_tfrc.tfrctx_rtt
#define tx_p				tx_tfrc.tfrctx_p
#define tx_t_rto			tx_tfrc.tfrctx_rto
#define tx_t_ipi			tx_tfrc.tfrctx_ipi
	u16				tx_s;
	enum ccid3_hc_tx_states		tx_state:8;
	u8				tx_last_win_count;
	ktime_t				tx_t_last_win_count;
	struct timer_list		tx_no_feedback_timer;
	ktime_t				tx_t_ld;
	ktime_t				tx_t_nom;
	u32				tx_delta;
	struct tfrc_tx_hist_entry	*tx_hist;
	struct ccid3_options_received	tx_options_received;
};

static inline struct ccid3_hc_tx_sock *ccid3_hc_tx_sk(const struct sock *sk)
{
	struct ccid3_hc_tx_sock *hctx = ccid_priv(dccp_sk(sk)->dccps_hc_tx_ccid);
	BUG_ON(hctx == NULL);
	return hctx;
}

/* TFRC receiver states */
enum ccid3_hc_rx_states {
	TFRC_RSTATE_NO_DATA = 1,
	TFRC_RSTATE_DATA,
	TFRC_RSTATE_TERM    = 127,
};

struct ccid3_hc_rx_sock {
	u8				rx_last_counter:4;
	enum ccid3_hc_rx_states		rx_state:8;
	u32				rx_bytes_recv;
	u32				rx_x_recv;
	u32				rx_rtt;
	ktime_t				rx_tstamp_last_feedback;
	struct tfrc_rx_hist		rx_hist;
	struct tfrc_loss_hist		rx_li_hist;
	u16				rx_s;
#define rx_pinv				rx_li_hist.i_mean
};

static inline struct ccid3_hc_rx_sock *ccid3_hc_rx_sk(const struct sock *sk)
{
	struct ccid3_hc_rx_sock *hcrx = ccid_priv(dccp_sk(sk)->dccps_hc_rx_ccid);
	BUG_ON(hcrx == NULL);
	return hcrx;
}

#endif /* _DCCP_CCID3_H_ */
