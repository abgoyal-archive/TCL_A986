

#ifndef __BFA_DEFS_ETHPORT_H__
#define __BFA_DEFS_ETHPORT_H__

#include <defs/bfa_defs_status.h>
#include <defs/bfa_defs_port.h>
#include <protocol/types.h>
#include <cna/pstats/phyport_defs.h>
#include <cna/pstats/ethport_defs.h>

struct bna_tx_info_s {
	u32    miniport_state;
	u32    adapter_state;
	u64    tx_count;
	u64    tx_wi;
	u64    tx_sg;
	u64    tx_tcp_chksum;
	u64    tx_udp_chksum;
	u64    tx_ip_chksum;
	u64    tx_lsov1;
	u64    tx_lsov2;
	u64    tx_max_sg_len ;
};

struct bna_rx_queue_info_s {
	u16    q_id ;
	u16    buf_size ;
	u16    buf_count ;
	u16    rsvd ;
	u64    rx_count ;
	u64    rx_dropped ;
	u64    rx_unsupported ;
	u64    rx_internal_err ;
	u64    rss_count ;
	u64    vlan_count ;
	u64    rx_tcp_chksum ;
	u64    rx_udp_chksum ;
	u64    rx_ip_chksum ;
	u64    rx_hds ;
};

struct bna_rx_q_set_s {
	u16    q_set_type;
	u32    miniport_state;
	u32    adapter_state;
	struct bna_rx_queue_info_s    rx_queue[2];
};

struct bna_port_stats_s {
	struct bna_tx_info_s   tx_stats;
	u16        qset_count ;
	struct bna_rx_q_set_s  rx_qset[8];
};

struct bfa_ethport_stats_s {
	struct bna_stats_txf	txf_stats[1];
	struct bna_stats_rxf	rxf_stats[1];
	struct bnad_drv_stats drv_stats;
};

enum bfa_ethport_aen_event {
	BFA_ETHPORT_AEN_LINKUP = 1, /*  Base Port Ethernet link up event */
	BFA_ETHPORT_AEN_LINKDOWN = 2, /*  Base Port Ethernet link down event */
	BFA_ETHPORT_AEN_ENABLE = 3, /*  Base Port Ethernet link enable event */
	BFA_ETHPORT_AEN_DISABLE = 4, /*  Base Port Ethernet link disable
				      * event */
};

struct bfa_ethport_aen_data_s {
	mac_t mac;	/*  MAC address of the physical port */
};


#endif /* __BFA_DEFS_ETHPORT_H__ */
