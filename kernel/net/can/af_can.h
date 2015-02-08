

#ifndef AF_CAN_H
#define AF_CAN_H

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/list.h>
#include <linux/rcupdate.h>
#include <linux/can.h>

/* af_can rx dispatcher structures */

struct receiver {
	struct hlist_node list;
	struct rcu_head rcu;
	canid_t can_id;
	canid_t mask;
	unsigned long matches;
	void (*func)(struct sk_buff *, void *);
	void *data;
	char *ident;
};

enum { RX_ERR, RX_ALL, RX_FIL, RX_INV, RX_EFF, RX_MAX };

/* per device receive filters linked at dev->ml_priv */
struct dev_rcv_lists {
	struct hlist_head rx[RX_MAX];
	struct hlist_head rx_sff[0x800];
	int remove_on_zero_entries;
	int entries;
};

/* statistic structures */

/* can be reset e.g. by can_init_stats() */
struct s_stats {
	unsigned long jiffies_init;

	unsigned long rx_frames;
	unsigned long tx_frames;
	unsigned long matches;

	unsigned long total_rx_rate;
	unsigned long total_tx_rate;
	unsigned long total_rx_match_ratio;

	unsigned long current_rx_rate;
	unsigned long current_tx_rate;
	unsigned long current_rx_match_ratio;

	unsigned long max_rx_rate;
	unsigned long max_tx_rate;
	unsigned long max_rx_match_ratio;

	unsigned long rx_frames_delta;
	unsigned long tx_frames_delta;
	unsigned long matches_delta;
};

/* persistent statistics */
struct s_pstats {
	unsigned long stats_reset;
	unsigned long user_reset;
	unsigned long rcv_entries;
	unsigned long rcv_entries_max;
};

/* function prototypes for the CAN networklayer procfs (proc.c) */
extern void can_init_proc(void);
extern void can_remove_proc(void);
extern void can_stat_update(unsigned long data);

/* structures and variables from af_can.c needed in proc.c for reading */
extern struct timer_list can_stattimer;    /* timer for statistics update */
extern struct s_stats    can_stats;        /* packet statistics */
extern struct s_pstats   can_pstats;       /* receive list statistics */
extern struct hlist_head can_rx_dev_list;  /* rx dispatcher structures */

#endif /* AF_CAN_H */
