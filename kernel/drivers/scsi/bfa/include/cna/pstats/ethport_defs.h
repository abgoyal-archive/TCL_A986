

#ifndef __ETHPORT_DEFS_H__
#define __ETHPORT_DEFS_H__

struct bnad_drv_stats {
	u64 netif_queue_stop;
	u64 netif_queue_wakeup;
	u64 tso4;
	u64 tso6;
	u64 tso_err;
	u64 tcpcsum_offload;
	u64 udpcsum_offload;
	u64 csum_help;
	u64 csum_help_err;

	u64 hw_stats_updates;
	u64 netif_rx_schedule;
	u64 netif_rx_complete;
	u64 netif_rx_dropped;
};
#endif
