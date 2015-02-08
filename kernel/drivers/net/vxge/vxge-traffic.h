
#ifndef VXGE_TRAFFIC_H
#define VXGE_TRAFFIC_H

#include "vxge-reg.h"
#include "vxge-version.h"

#define VXGE_HW_DTR_MAX_T_CODE		16
#define VXGE_HW_ALL_FOXES		0xFFFFFFFFFFFFFFFFULL
#define VXGE_HW_INTR_MASK_ALL		0xFFFFFFFFFFFFFFFFULL
#define	VXGE_HW_MAX_VIRTUAL_PATHS	17

#define VXGE_HW_MAC_MAX_MAC_PORT_ID	2

#define VXGE_HW_DEFAULT_32		0xffffffff
/* frames sizes */
#define VXGE_HW_HEADER_802_2_SIZE	3
#define VXGE_HW_HEADER_SNAP_SIZE	5
#define VXGE_HW_HEADER_VLAN_SIZE	4
#define VXGE_HW_MAC_HEADER_MAX_SIZE \
			(ETH_HLEN + \
			VXGE_HW_HEADER_802_2_SIZE + \
			VXGE_HW_HEADER_VLAN_SIZE + \
			VXGE_HW_HEADER_SNAP_SIZE)

/* 32bit alignments */
#define VXGE_HW_HEADER_ETHERNET_II_802_3_ALIGN		2
#define VXGE_HW_HEADER_802_2_SNAP_ALIGN			2
#define VXGE_HW_HEADER_802_2_ALIGN			3
#define VXGE_HW_HEADER_SNAP_ALIGN			1

#define VXGE_HW_L3_CKSUM_OK				0xFFFF
#define VXGE_HW_L4_CKSUM_OK				0xFFFF

/* Forward declarations */
struct __vxge_hw_device;
struct __vxge_hw_vpath_handle;
struct vxge_hw_vp_config;
struct __vxge_hw_virtualpath;
struct __vxge_hw_channel;
struct __vxge_hw_fifo;
struct __vxge_hw_ring;
struct vxge_hw_ring_attr;
struct vxge_hw_mempool;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*VXGE_HW_STATUS_H*/

#define VXGE_HW_EVENT_BASE			0
#define VXGE_LL_EVENT_BASE			100

enum vxge_hw_event {
	VXGE_HW_EVENT_UNKNOWN		= 0,
	/* HW events */
	VXGE_HW_EVENT_RESET_START	= VXGE_HW_EVENT_BASE + 1,
	VXGE_HW_EVENT_RESET_COMPLETE	= VXGE_HW_EVENT_BASE + 2,
	VXGE_HW_EVENT_LINK_DOWN		= VXGE_HW_EVENT_BASE + 3,
	VXGE_HW_EVENT_LINK_UP		= VXGE_HW_EVENT_BASE + 4,
	VXGE_HW_EVENT_ALARM_CLEARED	= VXGE_HW_EVENT_BASE + 5,
	VXGE_HW_EVENT_ECCERR		= VXGE_HW_EVENT_BASE + 6,
	VXGE_HW_EVENT_MRPCIM_ECCERR	= VXGE_HW_EVENT_BASE + 7,
	VXGE_HW_EVENT_FIFO_ERR		= VXGE_HW_EVENT_BASE + 8,
	VXGE_HW_EVENT_VPATH_ERR		= VXGE_HW_EVENT_BASE + 9,
	VXGE_HW_EVENT_CRITICAL_ERR	= VXGE_HW_EVENT_BASE + 10,
	VXGE_HW_EVENT_SERR		= VXGE_HW_EVENT_BASE + 11,
	VXGE_HW_EVENT_SRPCIM_SERR	= VXGE_HW_EVENT_BASE + 12,
	VXGE_HW_EVENT_MRPCIM_SERR	= VXGE_HW_EVENT_BASE + 13,
	VXGE_HW_EVENT_SLOT_FREEZE	= VXGE_HW_EVENT_BASE + 14,
};

#define VXGE_HW_SET_LEVEL(a, b) (((a) > (b)) ? (a) : (b))

struct vxge_hw_mempool_dma {
	dma_addr_t			addr;
	struct pci_dev *handle;
	struct pci_dev *acc_handle;
};


struct vxge_hw_mempool {

	void (*item_func_alloc)(
	struct vxge_hw_mempool *mempoolh,
	u32			memblock_index,
	struct vxge_hw_mempool_dma	*dma_object,
	u32			index,
	u32			is_last);

	void		*userdata;
	void		**memblocks_arr;
	void		**memblocks_priv_arr;
	struct vxge_hw_mempool_dma	*memblocks_dma_arr;
	struct __vxge_hw_device *devh;
	u32			memblock_size;
	u32			memblocks_max;
	u32			memblocks_allocated;
	u32			item_size;
	u32			items_max;
	u32			items_initial;
	u32			items_current;
	u32			items_per_memblock;
	void		**items_arr;
	u32			items_priv_size;
};

#define	VXGE_HW_MAX_INTR_PER_VP				4
#define	VXGE_HW_VPATH_INTR_TX				0
#define	VXGE_HW_VPATH_INTR_RX				1
#define	VXGE_HW_VPATH_INTR_EINTA			2
#define	VXGE_HW_VPATH_INTR_BMAP				3

#define VXGE_HW_BLOCK_SIZE				4096

struct vxge_hw_tim_intr_config {

	u32				intr_enable;
#define VXGE_HW_TIM_INTR_ENABLE				1
#define VXGE_HW_TIM_INTR_DISABLE				0
#define VXGE_HW_TIM_INTR_DEFAULT				0

	u32				btimer_val;
#define VXGE_HW_MIN_TIM_BTIMER_VAL				0
#define VXGE_HW_MAX_TIM_BTIMER_VAL				67108864
#define VXGE_HW_USE_FLASH_DEFAULT				0xffffffff

	u32				timer_ac_en;
#define VXGE_HW_TIM_TIMER_AC_ENABLE				1
#define VXGE_HW_TIM_TIMER_AC_DISABLE				0

	u32				timer_ci_en;
#define VXGE_HW_TIM_TIMER_CI_ENABLE				1
#define VXGE_HW_TIM_TIMER_CI_DISABLE				0

	u32				timer_ri_en;
#define VXGE_HW_TIM_TIMER_RI_ENABLE				1
#define VXGE_HW_TIM_TIMER_RI_DISABLE				0

	u32				rtimer_val;
#define VXGE_HW_MIN_TIM_RTIMER_VAL				0
#define VXGE_HW_MAX_TIM_RTIMER_VAL				67108864

	u32				util_sel;
#define VXGE_HW_TIM_UTIL_SEL_LEGACY_TX_NET_UTIL		17
#define VXGE_HW_TIM_UTIL_SEL_LEGACY_RX_NET_UTIL		18
#define VXGE_HW_TIM_UTIL_SEL_LEGACY_TX_RX_AVE_NET_UTIL		19
#define VXGE_HW_TIM_UTIL_SEL_PER_VPATH				63

	u32				ltimer_val;
#define VXGE_HW_MIN_TIM_LTIMER_VAL				0
#define VXGE_HW_MAX_TIM_LTIMER_VAL				67108864

	/* Line utilization interrupts */
	u32				urange_a;
#define VXGE_HW_MIN_TIM_URANGE_A				0
#define VXGE_HW_MAX_TIM_URANGE_A				100

	u32				uec_a;
#define VXGE_HW_MIN_TIM_UEC_A					0
#define VXGE_HW_MAX_TIM_UEC_A					65535

	u32				urange_b;
#define VXGE_HW_MIN_TIM_URANGE_B				0
#define VXGE_HW_MAX_TIM_URANGE_B				100

	u32				uec_b;
#define VXGE_HW_MIN_TIM_UEC_B					0
#define VXGE_HW_MAX_TIM_UEC_B					65535

	u32				urange_c;
#define VXGE_HW_MIN_TIM_URANGE_C				0
#define VXGE_HW_MAX_TIM_URANGE_C				100

	u32				uec_c;
#define VXGE_HW_MIN_TIM_UEC_C					0
#define VXGE_HW_MAX_TIM_UEC_C					65535

	u32				uec_d;
#define VXGE_HW_MIN_TIM_UEC_D					0
#define VXGE_HW_MAX_TIM_UEC_D					65535
};

#define	VXGE_HW_STATS_OP_READ					0
#define	VXGE_HW_STATS_OP_CLEAR_STAT				1
#define	VXGE_HW_STATS_OP_CLEAR_ALL_VPATH_STATS			2
#define	VXGE_HW_STATS_OP_CLEAR_ALL_STATS_OF_LOC			2
#define	VXGE_HW_STATS_OP_CLEAR_ALL_STATS			3

#define	VXGE_HW_STATS_LOC_AGGR					17
#define VXGE_HW_STATS_AGGRn_OFFSET				0x00720

#define VXGE_HW_STATS_VPATH_TX_OFFSET				0x0
#define VXGE_HW_STATS_VPATH_RX_OFFSET				0x00090

#define	VXGE_HW_STATS_VPATH_PROG_EVENT_VNUM0_OFFSET	   (0x001d0 >> 3)
#define	VXGE_HW_STATS_GET_VPATH_PROG_EVENT_VNUM0(bits) \
						vxge_bVALn(bits, 0, 32)

#define	VXGE_HW_STATS_GET_VPATH_PROG_EVENT_VNUM1(bits) \
						vxge_bVALn(bits, 32, 32)

#define	VXGE_HW_STATS_VPATH_PROG_EVENT_VNUM2_OFFSET	   (0x001d8 >> 3)
#define	VXGE_HW_STATS_GET_VPATH_PROG_EVENT_VNUM2(bits) \
						vxge_bVALn(bits, 0, 32)

#define	VXGE_HW_STATS_GET_VPATH_PROG_EVENT_VNUM3(bits) \
						vxge_bVALn(bits, 32, 32)

struct vxge_hw_xmac_aggr_stats {
/*0x000*/		u64	tx_frms;
/*0x008*/		u64	tx_data_octets;
/*0x010*/		u64	tx_mcast_frms;
/*0x018*/		u64	tx_bcast_frms;
/*0x020*/		u64	tx_discarded_frms;
/*0x028*/		u64	tx_errored_frms;
/*0x030*/		u64	rx_frms;
/*0x038*/		u64	rx_data_octets;
/*0x040*/		u64	rx_mcast_frms;
/*0x048*/		u64	rx_bcast_frms;
/*0x050*/		u64	rx_discarded_frms;
/*0x058*/		u64	rx_errored_frms;
/*0x060*/		u64	rx_unknown_slow_proto_frms;
} __packed;

struct vxge_hw_xmac_port_stats {
/*0x000*/		u64	tx_ttl_frms;
/*0x008*/		u64	tx_ttl_octets;
/*0x010*/		u64	tx_data_octets;
/*0x018*/		u64	tx_mcast_frms;
/*0x020*/		u64	tx_bcast_frms;
/*0x028*/		u64	tx_ucast_frms;
/*0x030*/		u64	tx_tagged_frms;
/*0x038*/		u64	tx_vld_ip;
/*0x040*/		u64	tx_vld_ip_octets;
/*0x048*/		u64	tx_icmp;
/*0x050*/		u64	tx_tcp;
/*0x058*/		u64	tx_rst_tcp;
/*0x060*/		u64	tx_udp;
/*0x068*/		u32	tx_parse_error;
/*0x06c*/		u32	tx_unknown_protocol;
/*0x070*/		u64	tx_pause_ctrl_frms;
/*0x078*/		u32	tx_marker_pdu_frms;
/*0x07c*/		u32	tx_lacpdu_frms;
/*0x080*/		u32	tx_drop_ip;
/*0x084*/		u32	tx_marker_resp_pdu_frms;
/*0x088*/		u32	tx_xgmii_char2_match;
/*0x08c*/		u32	tx_xgmii_char1_match;
/*0x090*/		u32	tx_xgmii_column2_match;
/*0x094*/		u32	tx_xgmii_column1_match;
/*0x098*/		u32	unused1;
/*0x09c*/		u16	tx_any_err_frms;
/*0x09e*/		u16	tx_drop_frms;
/*0x0a0*/		u64	rx_ttl_frms;
/*0x0a8*/		u64	rx_vld_frms;
/*0x0b0*/		u64	rx_offload_frms;
/*0x0b8*/		u64	rx_ttl_octets;
/*0x0c0*/		u64	rx_data_octets;
/*0x0c8*/		u64	rx_offload_octets;
/*0x0d0*/		u64	rx_vld_mcast_frms;
/*0x0d8*/		u64	rx_vld_bcast_frms;
/*0x0e0*/		u64	rx_accepted_ucast_frms;
/*0x0e8*/		u64	rx_accepted_nucast_frms;
/*0x0f0*/		u64	rx_tagged_frms;
/*0x0f8*/		u64	rx_long_frms;
/*0x100*/		u64	rx_usized_frms;
/*0x108*/		u64	rx_osized_frms;
/*0x110*/		u64	rx_frag_frms;
/*0x118*/		u64	rx_jabber_frms;
/*0x120*/		u64	rx_ttl_64_frms;
/*0x128*/		u64	rx_ttl_65_127_frms;
/*0x130*/		u64	rx_ttl_128_255_frms;
/*0x138*/		u64	rx_ttl_256_511_frms;
/*0x140*/		u64	rx_ttl_512_1023_frms;
/*0x148*/		u64	rx_ttl_1024_1518_frms;
/*0x150*/		u64	rx_ttl_1519_4095_frms;
/*0x158*/		u64	rx_ttl_4096_8191_frms;
/*0x160*/		u64	rx_ttl_8192_max_frms;
/*0x168*/		u64	rx_ttl_gt_max_frms;
/*0x170*/		u64	rx_ip;
/*0x178*/		u64	rx_accepted_ip;
/*0x180*/		u64	rx_ip_octets;
/*0x188*/		u64	rx_err_ip;
/*0x190*/		u64	rx_icmp;
/*0x198*/		u64	rx_tcp;
/*0x1a0*/		u64	rx_udp;
/*0x1a8*/		u64	rx_err_tcp;
/*0x1b0*/		u64	rx_pause_count;
/*0x1b8*/		u64	rx_pause_ctrl_frms;
/*0x1c0*/		u64	rx_unsup_ctrl_frms;
/*0x1c8*/		u64	rx_fcs_err_frms;
/*0x1d0*/		u64	rx_in_rng_len_err_frms;
/*0x1d8*/		u64	rx_out_rng_len_err_frms;
/*0x1e0*/		u64	rx_drop_frms;
/*0x1e8*/		u64	rx_discarded_frms;
/*0x1f0*/		u64	rx_drop_ip;
/*0x1f8*/		u64	rx_drop_udp;
/*0x200*/		u32	rx_marker_pdu_frms;
/*0x204*/		u32	rx_lacpdu_frms;
/*0x208*/		u32	rx_unknown_pdu_frms;
/*0x20c*/		u32	rx_marker_resp_pdu_frms;
/*0x210*/		u32	rx_fcs_discard;
/*0x214*/		u32	rx_illegal_pdu_frms;
/*0x218*/		u32	rx_switch_discard;
/*0x21c*/		u32	rx_len_discard;
/*0x220*/		u32	rx_rpa_discard;
/*0x224*/		u32	rx_l2_mgmt_discard;
/*0x228*/		u32	rx_rts_discard;
/*0x22c*/		u32	rx_trash_discard;
/*0x230*/		u32	rx_buff_full_discard;
/*0x234*/		u32	rx_red_discard;
/*0x238*/		u32	rx_xgmii_ctrl_err_cnt;
/*0x23c*/		u32	rx_xgmii_data_err_cnt;
/*0x240*/		u32	rx_xgmii_char1_match;
/*0x244*/		u32	rx_xgmii_err_sym;
/*0x248*/		u32	rx_xgmii_column1_match;
/*0x24c*/		u32	rx_xgmii_char2_match;
/*0x250*/		u32	rx_local_fault;
/*0x254*/		u32	rx_xgmii_column2_match;
/*0x258*/		u32	rx_jettison;
/*0x25c*/		u32	rx_remote_fault;
} __packed;

struct vxge_hw_xmac_vpath_tx_stats {
	u64	tx_ttl_eth_frms;
	u64	tx_ttl_eth_octets;
	u64	tx_data_octets;
	u64	tx_mcast_frms;
	u64	tx_bcast_frms;
	u64	tx_ucast_frms;
	u64	tx_tagged_frms;
	u64	tx_vld_ip;
	u64	tx_vld_ip_octets;
	u64	tx_icmp;
	u64	tx_tcp;
	u64	tx_rst_tcp;
	u64	tx_udp;
	u32	tx_unknown_protocol;
	u32	tx_lost_ip;
	u32	unused1;
	u32	tx_parse_error;
	u64	tx_tcp_offload;
	u64	tx_retx_tcp_offload;
	u64	tx_lost_ip_offload;
} __packed;

struct vxge_hw_xmac_vpath_rx_stats {
	u64	rx_ttl_eth_frms;
	u64	rx_vld_frms;
	u64	rx_offload_frms;
	u64	rx_ttl_eth_octets;
	u64	rx_data_octets;
	u64	rx_offload_octets;
	u64	rx_vld_mcast_frms;
	u64	rx_vld_bcast_frms;
	u64	rx_accepted_ucast_frms;
	u64	rx_accepted_nucast_frms;
	u64	rx_tagged_frms;
	u64	rx_long_frms;
	u64	rx_usized_frms;
	u64	rx_osized_frms;
	u64	rx_frag_frms;
	u64	rx_jabber_frms;
	u64	rx_ttl_64_frms;
	u64	rx_ttl_65_127_frms;
	u64	rx_ttl_128_255_frms;
	u64	rx_ttl_256_511_frms;
	u64	rx_ttl_512_1023_frms;
	u64	rx_ttl_1024_1518_frms;
	u64	rx_ttl_1519_4095_frms;
	u64	rx_ttl_4096_8191_frms;
	u64	rx_ttl_8192_max_frms;
	u64	rx_ttl_gt_max_frms;
	u64	rx_ip;
	u64	rx_accepted_ip;
	u64	rx_ip_octets;
	u64	rx_err_ip;
	u64	rx_icmp;
	u64	rx_tcp;
	u64	rx_udp;
	u64	rx_err_tcp;
	u64	rx_lost_frms;
	u64	rx_lost_ip;
	u64	rx_lost_ip_offload;
	u16	rx_various_discard;
	u16	rx_sleep_discard;
	u16	rx_red_discard;
	u16	rx_queue_full_discard;
	u64	rx_mpa_ok_frms;
} __packed;

struct vxge_hw_xmac_stats {
	struct vxge_hw_xmac_aggr_stats
				aggr_stats[VXGE_HW_MAC_MAX_MAC_PORT_ID];
	struct vxge_hw_xmac_port_stats
				port_stats[VXGE_HW_MAC_MAX_MAC_PORT_ID+1];
	struct vxge_hw_xmac_vpath_tx_stats
				vpath_tx_stats[VXGE_HW_MAX_VIRTUAL_PATHS];
	struct vxge_hw_xmac_vpath_rx_stats
				vpath_rx_stats[VXGE_HW_MAX_VIRTUAL_PATHS];
};

struct vxge_hw_vpath_stats_hw_info {
/*0x000*/	u32 ini_num_mwr_sent;
/*0x004*/	u32 unused1;
/*0x008*/	u32 ini_num_mrd_sent;
/*0x00c*/	u32 unused2;
/*0x010*/	u32 ini_num_cpl_rcvd;
/*0x014*/	u32 unused3;
/*0x018*/	u64 ini_num_mwr_byte_sent;
/*0x020*/	u64 ini_num_cpl_byte_rcvd;
/*0x028*/	u32 wrcrdtarb_xoff;
/*0x02c*/	u32 unused4;
/*0x030*/	u32 rdcrdtarb_xoff;
/*0x034*/	u32 unused5;
/*0x038*/	u32 vpath_genstats_count0;
/*0x03c*/	u32 vpath_genstats_count1;
/*0x040*/	u32 vpath_genstats_count2;
/*0x044*/	u32 vpath_genstats_count3;
/*0x048*/	u32 vpath_genstats_count4;
/*0x04c*/	u32 unused6;
/*0x050*/	u32 vpath_genstats_count5;
/*0x054*/	u32 unused7;
/*0x058*/	struct vxge_hw_xmac_vpath_tx_stats tx_stats;
/*0x0e8*/	struct vxge_hw_xmac_vpath_rx_stats rx_stats;
/*0x220*/	u64 unused9;
/*0x228*/	u32 prog_event_vnum1;
/*0x22c*/	u32 prog_event_vnum0;
/*0x230*/	u32 prog_event_vnum3;
/*0x234*/	u32 prog_event_vnum2;
/*0x238*/	u16 rx_multi_cast_frame_discard;
/*0x23a*/	u8 unused10[6];
/*0x240*/	u32 rx_frm_transferred;
/*0x244*/	u32 unused11;
/*0x248*/	u16 rxd_returned;
/*0x24a*/	u8 unused12[6];
/*0x252*/	u16 rx_mpa_len_fail_frms;
/*0x254*/	u16 rx_mpa_mrk_fail_frms;
/*0x256*/	u16 rx_mpa_crc_fail_frms;
/*0x258*/	u16 rx_permitted_frms;
/*0x25c*/	u64 rx_vp_reset_discarded_frms;
/*0x25e*/	u64 rx_wol_frms;
/*0x260*/	u64 tx_vp_reset_discarded_frms;
} __packed;


struct vxge_hw_device_stats_mrpcim_info {
/*0x0000*/	u32	pic_ini_rd_drop;
/*0x0004*/	u32	pic_ini_wr_drop;
/*0x0008*/	struct {
	/*0x0000*/	u32	pic_wrcrdtarb_ph_crdt_depleted;
	/*0x0004*/	u32	unused1;
		} pic_wrcrdtarb_ph_crdt_depleted_vplane[17];
/*0x0090*/	struct {
	/*0x0000*/	u32	pic_wrcrdtarb_pd_crdt_depleted;
	/*0x0004*/	u32	unused2;
		} pic_wrcrdtarb_pd_crdt_depleted_vplane[17];
/*0x0118*/	struct {
	/*0x0000*/	u32	pic_rdcrdtarb_nph_crdt_depleted;
	/*0x0004*/	u32	unused3;
		} pic_rdcrdtarb_nph_crdt_depleted_vplane[17];
/*0x01a0*/	u32	pic_ini_rd_vpin_drop;
/*0x01a4*/	u32	pic_ini_wr_vpin_drop;
/*0x01a8*/	u32	pic_genstats_count0;
/*0x01ac*/	u32	pic_genstats_count1;
/*0x01b0*/	u32	pic_genstats_count2;
/*0x01b4*/	u32	pic_genstats_count3;
/*0x01b8*/	u32	pic_genstats_count4;
/*0x01bc*/	u32	unused4;
/*0x01c0*/	u32	pic_genstats_count5;
/*0x01c4*/	u32	unused5;
/*0x01c8*/	u32	pci_rstdrop_cpl;
/*0x01cc*/	u32	pci_rstdrop_msg;
/*0x01d0*/	u32	pci_rstdrop_client1;
/*0x01d4*/	u32	pci_rstdrop_client0;
/*0x01d8*/	u32	pci_rstdrop_client2;
/*0x01dc*/	u32	unused6;
/*0x01e0*/	struct {
	/*0x0000*/	u16	unused7;
	/*0x0002*/	u16	pci_depl_cplh;
	/*0x0004*/	u16	pci_depl_nph;
	/*0x0006*/	u16	pci_depl_ph;
		} pci_depl_h_vplane[17];
/*0x0268*/	struct {
	/*0x0000*/	u16	unused8;
	/*0x0002*/	u16	pci_depl_cpld;
	/*0x0004*/	u16	pci_depl_npd;
	/*0x0006*/	u16	pci_depl_pd;
		} pci_depl_d_vplane[17];
/*0x02f0*/	struct vxge_hw_xmac_port_stats xgmac_port[3];
/*0x0a10*/	struct vxge_hw_xmac_aggr_stats xgmac_aggr[2];
/*0x0ae0*/	u64	xgmac_global_prog_event_gnum0;
/*0x0ae8*/	u64	xgmac_global_prog_event_gnum1;
/*0x0af0*/	u64	unused7;
/*0x0af8*/	u64	unused8;
/*0x0b00*/	u64	unused9;
/*0x0b08*/	u64	unused10;
/*0x0b10*/	u32	unused11;
/*0x0b14*/	u32	xgmac_tx_permitted_frms;
/*0x0b18*/	u32	unused12;
/*0x0b1c*/	u8	unused13;
/*0x0b1d*/	u8	xgmac_port2_tx_any_frms;
/*0x0b1e*/	u8	xgmac_port1_tx_any_frms;
/*0x0b1f*/	u8	xgmac_port0_tx_any_frms;
/*0x0b20*/	u32	unused14;
/*0x0b24*/	u8	unused15;
/*0x0b25*/	u8	xgmac_port2_rx_any_frms;
/*0x0b26*/	u8	xgmac_port1_rx_any_frms;
/*0x0b27*/	u8	xgmac_port0_rx_any_frms;
} __packed;

struct vxge_hw_device_stats_hw_info {
	struct vxge_hw_vpath_stats_hw_info
		*vpath_info[VXGE_HW_MAX_VIRTUAL_PATHS];
	struct vxge_hw_vpath_stats_hw_info
		vpath_info_sav[VXGE_HW_MAX_VIRTUAL_PATHS];
};

struct vxge_hw_vpath_stats_sw_common_info {
	u32	full_cnt;
	u32	usage_cnt;
	u32	usage_max;
	u32	reserve_free_swaps_cnt;
	u32 total_compl_cnt;
};

struct vxge_hw_vpath_stats_sw_fifo_info {
	struct vxge_hw_vpath_stats_sw_common_info common_stats;
	u32 total_posts;
	u32 total_buffers;
	u32 txd_t_code_err_cnt[VXGE_HW_DTR_MAX_T_CODE];
};

struct vxge_hw_vpath_stats_sw_ring_info {
	struct vxge_hw_vpath_stats_sw_common_info common_stats;
	u32 rxd_t_code_err_cnt[VXGE_HW_DTR_MAX_T_CODE];

};

struct vxge_hw_vpath_stats_sw_err {
	u32	unknown_alarms;
	u32	network_sustained_fault;
	u32	network_sustained_ok;
	u32	kdfcctl_fifo0_overwrite;
	u32	kdfcctl_fifo0_poison;
	u32	kdfcctl_fifo0_dma_error;
	u32	dblgen_fifo0_overflow;
	u32	statsb_pif_chain_error;
	u32	statsb_drop_timeout;
	u32	target_illegal_access;
	u32	ini_serr_det;
	u32	prc_ring_bumps;
	u32	prc_rxdcm_sc_err;
	u32	prc_rxdcm_sc_abort;
	u32	prc_quanta_size_err;
};

struct vxge_hw_vpath_stats_sw_info {
	u32    soft_reset_cnt;
	struct vxge_hw_vpath_stats_sw_err	error_stats;
	struct vxge_hw_vpath_stats_sw_ring_info	ring_stats;
	struct vxge_hw_vpath_stats_sw_fifo_info	fifo_stats;
};

struct vxge_hw_device_stats_sw_info {
	u32	not_traffic_intr_cnt;
	u32	traffic_intr_cnt;
	u32	total_intr_cnt;
	u32	soft_reset_cnt;
	struct vxge_hw_vpath_stats_sw_info
		vpath_info[VXGE_HW_MAX_VIRTUAL_PATHS];
};

struct vxge_hw_device_stats_sw_err {
	u32     vpath_alarms;
};

struct vxge_hw_device_stats {
	/* handles */
	struct __vxge_hw_device *devh;

	/* HW device hardware statistics */
	struct vxge_hw_device_stats_hw_info	hw_dev_info_stats;

	/* HW device "soft" stats */
	struct vxge_hw_device_stats_sw_err   sw_dev_err_stats;
	struct vxge_hw_device_stats_sw_info  sw_dev_info_stats;

};

enum vxge_hw_status vxge_hw_device_hw_stats_enable(
			struct __vxge_hw_device *devh);

enum vxge_hw_status vxge_hw_device_stats_get(
			struct __vxge_hw_device *devh,
			struct vxge_hw_device_stats_hw_info *hw_stats);

enum vxge_hw_status vxge_hw_driver_stats_get(
			struct __vxge_hw_device *devh,
			struct vxge_hw_device_stats_sw_info *sw_stats);

enum vxge_hw_status vxge_hw_mrpcim_stats_enable(struct __vxge_hw_device *devh);

enum vxge_hw_status vxge_hw_mrpcim_stats_disable(struct __vxge_hw_device *devh);

enum vxge_hw_status
vxge_hw_mrpcim_stats_access(
	struct __vxge_hw_device *devh,
	u32 operation,
	u32 location,
	u32 offset,
	u64 *stat);

enum vxge_hw_status
vxge_hw_device_xmac_aggr_stats_get(struct __vxge_hw_device *devh, u32 port,
				   struct vxge_hw_xmac_aggr_stats *aggr_stats);

enum vxge_hw_status
vxge_hw_device_xmac_port_stats_get(struct __vxge_hw_device *devh, u32 port,
				   struct vxge_hw_xmac_port_stats *port_stats);

enum vxge_hw_status
vxge_hw_device_xmac_stats_get(struct __vxge_hw_device *devh,
			      struct vxge_hw_xmac_stats *xmac_stats);

enum vxge_hw_mgmt_reg_type {
	vxge_hw_mgmt_reg_type_legacy = 0,
	vxge_hw_mgmt_reg_type_toc = 1,
	vxge_hw_mgmt_reg_type_common = 2,
	vxge_hw_mgmt_reg_type_mrpcim = 3,
	vxge_hw_mgmt_reg_type_srpcim = 4,
	vxge_hw_mgmt_reg_type_vpmgmt = 5,
	vxge_hw_mgmt_reg_type_vpath = 6
};

enum vxge_hw_status
vxge_hw_mgmt_reg_read(struct __vxge_hw_device *devh,
		      enum vxge_hw_mgmt_reg_type type,
		      u32 index,
		      u32 offset,
		      u64 *value);

enum vxge_hw_status
vxge_hw_mgmt_reg_write(struct __vxge_hw_device *devh,
		      enum vxge_hw_mgmt_reg_type type,
		      u32 index,
		      u32 offset,
		      u64 value);

enum vxge_hw_rxd_state {
	VXGE_HW_RXD_STATE_NONE		= 0,
	VXGE_HW_RXD_STATE_AVAIL		= 1,
	VXGE_HW_RXD_STATE_POSTED	= 2,
	VXGE_HW_RXD_STATE_FREED		= 3
};

struct vxge_hw_ring_rxd_info {
	u32	syn_flag;
	u32	is_icmp;
	u32	fast_path_eligible;
	u32	l3_cksum_valid;
	u32	l3_cksum;
	u32	l4_cksum_valid;
	u32	l4_cksum;
	u32	frame;
	u32	proto;
	u32	is_vlan;
	u32	vlan;
	u32	rth_bucket;
	u32	rth_it_hit;
	u32	rth_spdm_hit;
	u32	rth_hash_type;
	u32	rth_value;
};
enum vxge_hw_ring_tcode {
	VXGE_HW_RING_T_CODE_OK				= 0x0,
	VXGE_HW_RING_T_CODE_L3_CKSUM_MISMATCH		= 0x1,
	VXGE_HW_RING_T_CODE_L4_CKSUM_MISMATCH		= 0x2,
	VXGE_HW_RING_T_CODE_L3_L4_CKSUM_MISMATCH	= 0x3,
	VXGE_HW_RING_T_CODE_L3_PKT_ERR			= 0x5,
	VXGE_HW_RING_T_CODE_L2_FRM_ERR			= 0x6,
	VXGE_HW_RING_T_CODE_BUF_SIZE_ERR		= 0x7,
	VXGE_HW_RING_T_CODE_INT_ECC_ERR			= 0x8,
	VXGE_HW_RING_T_CODE_BENIGN_OVFLOW		= 0x9,
	VXGE_HW_RING_T_CODE_ZERO_LEN_BUFF		= 0xA,
	VXGE_HW_RING_T_CODE_FRM_DROP			= 0xC,
	VXGE_HW_RING_T_CODE_UNUSED			= 0xE,
	VXGE_HW_RING_T_CODE_MULTI_ERR			= 0xF
};

enum vxge_hw_ring_hash_type {
	VXGE_HW_RING_HASH_TYPE_NONE			= 0x0,
	VXGE_HW_RING_HASH_TYPE_TCP_IPV4		= 0x1,
	VXGE_HW_RING_HASH_TYPE_UDP_IPV4		= 0x2,
	VXGE_HW_RING_HASH_TYPE_IPV4			= 0x3,
	VXGE_HW_RING_HASH_TYPE_TCP_IPV6		= 0x4,
	VXGE_HW_RING_HASH_TYPE_UDP_IPV6		= 0x5,
	VXGE_HW_RING_HASH_TYPE_IPV6			= 0x6,
	VXGE_HW_RING_HASH_TYPE_TCP_IPV6_EX	= 0x7,
	VXGE_HW_RING_HASH_TYPE_UDP_IPV6_EX	= 0x8,
	VXGE_HW_RING_HASH_TYPE_IPV6_EX		= 0x9
};

enum vxge_hw_status vxge_hw_ring_rxd_reserve(
	struct __vxge_hw_ring *ring_handle,
	void **rxdh);

void
vxge_hw_ring_rxd_pre_post(
	struct __vxge_hw_ring *ring_handle,
	void *rxdh);

void
vxge_hw_ring_rxd_post_post(
	struct __vxge_hw_ring *ring_handle,
	void *rxdh);

enum vxge_hw_status
vxge_hw_ring_replenish(struct __vxge_hw_ring *ring_handle);

void
vxge_hw_ring_rxd_post_post_wmb(
	struct __vxge_hw_ring *ring_handle,
	void *rxdh);

void vxge_hw_ring_rxd_post(
	struct __vxge_hw_ring *ring_handle,
	void *rxdh);

enum vxge_hw_status vxge_hw_ring_rxd_next_completed(
	struct __vxge_hw_ring *ring_handle,
	void **rxdh,
	u8 *t_code);

enum vxge_hw_status vxge_hw_ring_handle_tcode(
	struct __vxge_hw_ring *ring_handle,
	void *rxdh,
	u8 t_code);

void vxge_hw_ring_rxd_free(
	struct __vxge_hw_ring *ring_handle,
	void *rxdh);

enum vxge_hw_frame_proto {
	VXGE_HW_FRAME_PROTO_VLAN_TAGGED = 0x80,
	VXGE_HW_FRAME_PROTO_IPV4		= 0x10,
	VXGE_HW_FRAME_PROTO_IPV6		= 0x08,
	VXGE_HW_FRAME_PROTO_IP_FRAG		= 0x04,
	VXGE_HW_FRAME_PROTO_TCP			= 0x02,
	VXGE_HW_FRAME_PROTO_UDP			= 0x01,
	VXGE_HW_FRAME_PROTO_TCP_OR_UDP	= (VXGE_HW_FRAME_PROTO_TCP | \
						   VXGE_HW_FRAME_PROTO_UDP)
};

enum vxge_hw_fifo_gather_code {
	VXGE_HW_FIFO_GATHER_CODE_FIRST		= 0x2,
	VXGE_HW_FIFO_GATHER_CODE_MIDDLE		= 0x0,
	VXGE_HW_FIFO_GATHER_CODE_LAST		= 0x1,
	VXGE_HW_FIFO_GATHER_CODE_FIRST_LAST	= 0x3
};

enum vxge_hw_fifo_tcode {
	VXGE_HW_FIFO_T_CODE_OK			= 0x0,
	VXGE_HW_FIFO_T_CODE_PCI_READ_CORRUPT	= 0x1,
	VXGE_HW_FIFO_T_CODE_PCI_READ_FAIL	= 0x2,
	VXGE_HW_FIFO_T_CODE_INVALID_MSS		= 0x3,
	VXGE_HW_FIFO_T_CODE_LSO_ERROR		= 0x4,
	VXGE_HW_FIFO_T_CODE_UNUSED		= 0x7,
	VXGE_HW_FIFO_T_CODE_MULTI_ERROR		= 0x8
};

enum vxge_hw_status vxge_hw_fifo_txdl_reserve(
	struct __vxge_hw_fifo *fifoh,
	void **txdlh,
	void **txdl_priv);

void vxge_hw_fifo_txdl_buffer_set(
			struct __vxge_hw_fifo *fifo_handle,
			void *txdlh,
			u32 frag_idx,
			dma_addr_t dma_pointer,
			u32 size);

void vxge_hw_fifo_txdl_post(
			struct __vxge_hw_fifo *fifo_handle,
			void *txdlh);

u32 vxge_hw_fifo_free_txdl_count_get(
			struct __vxge_hw_fifo *fifo_handle);

enum vxge_hw_status vxge_hw_fifo_txdl_next_completed(
	struct __vxge_hw_fifo *fifoh,
	void **txdlh,
	enum vxge_hw_fifo_tcode *t_code);

enum vxge_hw_status vxge_hw_fifo_handle_tcode(
	struct __vxge_hw_fifo *fifoh,
	void *txdlh,
	enum vxge_hw_fifo_tcode t_code);

void vxge_hw_fifo_txdl_free(
	struct __vxge_hw_fifo *fifoh,
	void *txdlh);


#define VXGE_HW_RING_NEXT_BLOCK_POINTER_OFFSET	(VXGE_HW_BLOCK_SIZE-8)
#define VXGE_HW_RING_MEMBLOCK_IDX_OFFSET		(VXGE_HW_BLOCK_SIZE-16)

struct __vxge_hw_ring_rxd_priv {
	dma_addr_t	dma_addr;
	struct pci_dev *dma_handle;
	ptrdiff_t	dma_offset;
#ifdef VXGE_DEBUG_ASSERT
	struct vxge_hw_mempool_dma	*dma_object;
#endif
};

/* ========================= RING PRIVATE API ============================= */
u64
__vxge_hw_ring_first_block_address_get(
	struct __vxge_hw_ring *ringh);

enum vxge_hw_status
__vxge_hw_ring_create(
	struct __vxge_hw_vpath_handle *vpath_handle,
	struct vxge_hw_ring_attr *attr);

enum vxge_hw_status
__vxge_hw_ring_abort(
	struct __vxge_hw_ring *ringh);

enum vxge_hw_status
__vxge_hw_ring_reset(
	struct __vxge_hw_ring *ringh);

enum vxge_hw_status
__vxge_hw_ring_delete(
	struct __vxge_hw_vpath_handle *vpath_handle);

/* ========================= FIFO PRIVATE API ============================= */

struct vxge_hw_fifo_attr;

enum vxge_hw_status
__vxge_hw_fifo_create(
	struct __vxge_hw_vpath_handle *vpath_handle,
	struct vxge_hw_fifo_attr *attr);

enum vxge_hw_status
__vxge_hw_fifo_abort(
	struct __vxge_hw_fifo *fifoh);

enum vxge_hw_status
__vxge_hw_fifo_reset(
	struct __vxge_hw_fifo *ringh);

enum vxge_hw_status
__vxge_hw_fifo_delete(
	struct __vxge_hw_vpath_handle *vpath_handle);

struct vxge_hw_mempool_cbs {
	void (*item_func_alloc)(
			struct vxge_hw_mempool *mempoolh,
			u32			memblock_index,
			struct vxge_hw_mempool_dma	*dma_object,
			u32			index,
			u32			is_last);
};

void
__vxge_hw_mempool_destroy(
	struct vxge_hw_mempool *mempool);

#define VXGE_HW_VIRTUAL_PATH_HANDLE(vpath)				\
		((struct __vxge_hw_vpath_handle *)(vpath)->vpath_handles.next)

enum vxge_hw_status
__vxge_hw_vpath_rts_table_get(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u32			action,
	u32			rts_table,
	u32			offset,
	u64			*data1,
	u64			*data2);

enum vxge_hw_status
__vxge_hw_vpath_rts_table_set(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u32			action,
	u32			rts_table,
	u32			offset,
	u64			data1,
	u64			data2);

enum vxge_hw_status
__vxge_hw_vpath_reset(
	struct __vxge_hw_device *devh,
	u32			vp_id);

enum vxge_hw_status
__vxge_hw_vpath_sw_reset(
	struct __vxge_hw_device *devh,
	u32			vp_id);

enum vxge_hw_status
__vxge_hw_vpath_enable(
	struct __vxge_hw_device *devh,
	u32			vp_id);

void
__vxge_hw_vpath_prc_configure(
	struct __vxge_hw_device *devh,
	u32			vp_id);

enum vxge_hw_status
__vxge_hw_vpath_kdfc_configure(
	struct __vxge_hw_device *devh,
	u32			vp_id);

enum vxge_hw_status
__vxge_hw_vpath_mac_configure(
	struct __vxge_hw_device *devh,
	u32			vp_id);

enum vxge_hw_status
__vxge_hw_vpath_tim_configure(
	struct __vxge_hw_device *devh,
	u32			vp_id);

enum vxge_hw_status
__vxge_hw_vpath_initialize(
	struct __vxge_hw_device *devh,
	u32			vp_id);

enum vxge_hw_status
__vxge_hw_vp_initialize(
	struct __vxge_hw_device *devh,
	u32			vp_id,
	struct vxge_hw_vp_config	*config);

void
__vxge_hw_vp_terminate(
	struct __vxge_hw_device *devh,
	u32			vp_id);

enum vxge_hw_status
__vxge_hw_vpath_alarm_process(
	struct __vxge_hw_virtualpath	*vpath,
	u32			skip_alarms);

void vxge_hw_device_intr_enable(
	struct __vxge_hw_device *devh);

u32 vxge_hw_device_set_intr_type(struct __vxge_hw_device *devh, u32 intr_mode);

void vxge_hw_device_intr_disable(
	struct __vxge_hw_device *devh);

void vxge_hw_device_mask_all(
	struct __vxge_hw_device *devh);

void vxge_hw_device_unmask_all(
	struct __vxge_hw_device *devh);

enum vxge_hw_status vxge_hw_device_begin_irq(
	struct __vxge_hw_device *devh,
	u32 skip_alarms,
	u64 *reason);

void vxge_hw_device_clear_tx_rx(
	struct __vxge_hw_device *devh);


u32 vxge_hw_vpath_id(
	struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_vpath_mac_addr_add_mode {
	VXGE_HW_VPATH_MAC_ADDR_ADD_DUPLICATE = 0,
	VXGE_HW_VPATH_MAC_ADDR_DISCARD_DUPLICATE = 1,
	VXGE_HW_VPATH_MAC_ADDR_REPLACE_DUPLICATE = 2
};

enum vxge_hw_status
vxge_hw_vpath_mac_addr_add(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u8 (macaddr)[ETH_ALEN],
	u8 (macaddr_mask)[ETH_ALEN],
	enum vxge_hw_vpath_mac_addr_add_mode duplicate_mode);

enum vxge_hw_status
vxge_hw_vpath_mac_addr_get(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u8 (macaddr)[ETH_ALEN],
	u8 (macaddr_mask)[ETH_ALEN]);

enum vxge_hw_status
vxge_hw_vpath_mac_addr_get_next(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u8 (macaddr)[ETH_ALEN],
	u8 (macaddr_mask)[ETH_ALEN]);

enum vxge_hw_status
vxge_hw_vpath_mac_addr_delete(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u8 (macaddr)[ETH_ALEN],
	u8 (macaddr_mask)[ETH_ALEN]);

enum vxge_hw_status
vxge_hw_vpath_vid_add(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u64			vid);

enum vxge_hw_status
vxge_hw_vpath_vid_get(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u64			*vid);

enum vxge_hw_status
vxge_hw_vpath_vid_get_next(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u64			*vid);

enum vxge_hw_status
vxge_hw_vpath_vid_delete(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u64			vid);

enum vxge_hw_status
vxge_hw_vpath_etype_add(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u64			etype);

enum vxge_hw_status
vxge_hw_vpath_etype_get(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u64			*etype);

enum vxge_hw_status
vxge_hw_vpath_etype_get_next(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u64			*etype);

enum vxge_hw_status
vxge_hw_vpath_etype_delete(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u64			etype);

enum vxge_hw_status vxge_hw_vpath_promisc_enable(
	struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_status vxge_hw_vpath_promisc_disable(
	struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_status vxge_hw_vpath_bcast_enable(
	struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_status vxge_hw_vpath_mcast_enable(
	struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_status vxge_hw_vpath_mcast_disable(
	struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_status vxge_hw_vpath_poll_rx(
	struct __vxge_hw_ring *ringh);

enum vxge_hw_status vxge_hw_vpath_poll_tx(
	struct __vxge_hw_fifo *fifoh,
	struct sk_buff ***skb_ptr, int nr_skb, int *more);

enum vxge_hw_status vxge_hw_vpath_alarm_process(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u32 skip_alarms);

void
vxge_hw_vpath_msix_set(struct __vxge_hw_vpath_handle *vpath_handle,
		       int *tim_msix_id, int alarm_msix_id);

void
vxge_hw_vpath_msix_mask(struct __vxge_hw_vpath_handle *vpath_handle,
			int msix_id);

void vxge_hw_device_flush_io(struct __vxge_hw_device *devh);

void
vxge_hw_vpath_msix_clear(struct __vxge_hw_vpath_handle *vpath_handle,
			 int msix_id);

void
vxge_hw_vpath_msix_unmask(struct __vxge_hw_vpath_handle *vpath_handle,
			  int msix_id);

void
vxge_hw_vpath_msix_mask_all(struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_status vxge_hw_vpath_intr_enable(
				struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_status vxge_hw_vpath_intr_disable(
				struct __vxge_hw_vpath_handle *vpath_handle);

void vxge_hw_vpath_inta_mask_tx_rx(
	struct __vxge_hw_vpath_handle *vpath_handle);

void vxge_hw_vpath_inta_unmask_tx_rx(
	struct __vxge_hw_vpath_handle *vpath_handle);

void
vxge_hw_channel_msix_mask(struct __vxge_hw_channel *channelh, int msix_id);

void
vxge_hw_channel_msix_unmask(struct __vxge_hw_channel *channelh, int msix_id);

enum vxge_hw_status
vxge_hw_channel_dtr_alloc(struct __vxge_hw_channel *channel, void **dtrh);

void
vxge_hw_channel_dtr_post(struct __vxge_hw_channel *channel, void *dtrh);

void
vxge_hw_channel_dtr_try_complete(struct __vxge_hw_channel *channel,
				 void **dtrh);

void
vxge_hw_channel_dtr_complete(struct __vxge_hw_channel *channel);

void
vxge_hw_channel_dtr_free(struct __vxge_hw_channel *channel, void *dtrh);

int
vxge_hw_channel_dtr_count(struct __vxge_hw_channel *channel);
void
vxge_hw_vpath_tti_ci_set(struct __vxge_hw_device *hldev, u32 vp_id);

/* ========================== PRIVATE API ================================= */

enum vxge_hw_status
__vxge_hw_device_handle_link_up_ind(struct __vxge_hw_device *hldev);

enum vxge_hw_status
__vxge_hw_device_handle_link_down_ind(struct __vxge_hw_device *hldev);

enum vxge_hw_status
__vxge_hw_device_handle_error(
		struct __vxge_hw_device *hldev,
		u32 vp_id,
		enum vxge_hw_event type);

#endif
