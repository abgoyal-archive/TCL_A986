
#ifndef VXGE_CONFIG_H
#define VXGE_CONFIG_H
#include <linux/list.h>
#include <linux/slab.h>

#ifndef VXGE_CACHE_LINE_SIZE
#define VXGE_CACHE_LINE_SIZE 128
#endif

#define vxge_os_vaprintf(level, mask, fmt, ...) { \
	char buff[255]; \
		snprintf(buff, 255, fmt, __VA_ARGS__); \
		printk(buff); \
		printk("\n"); \
}

#ifndef VXGE_ALIGN
#define VXGE_ALIGN(adrs, size) \
	(((size) - (((u64)adrs) & ((size)-1))) & ((size)-1))
#endif

#define VXGE_HW_MIN_MTU				68
#define VXGE_HW_MAX_MTU				9600
#define VXGE_HW_DEFAULT_MTU			1500

#ifdef VXGE_DEBUG_ASSERT

#define vxge_assert(test) { \
	if (!(test)) \
		vxge_os_bug("bad cond: "#test" at %s:%d\n", \
				__FILE__, __LINE__); }
#else
#define vxge_assert(test)
#endif /* end of VXGE_DEBUG_ASSERT */

enum vxge_debug_level {
	VXGE_NONE   = 0,
	VXGE_TRACE  = 1,
	VXGE_ERR    = 2
};

#define NULL_VPID					0xFFFFFFFF
#ifdef CONFIG_VXGE_DEBUG_TRACE_ALL
#define VXGE_DEBUG_MODULE_MASK  0xffffffff
#define VXGE_DEBUG_TRACE_MASK   0xffffffff
#define VXGE_DEBUG_ERR_MASK     0xffffffff
#define VXGE_DEBUG_MASK         0x000001ff
#else
#define VXGE_DEBUG_MODULE_MASK  0x20000000
#define VXGE_DEBUG_TRACE_MASK   0x20000000
#define VXGE_DEBUG_ERR_MASK     0x20000000
#define VXGE_DEBUG_MASK         0x00000001
#endif

#define	VXGE_COMPONENT_LL				0x20000000
#define	VXGE_COMPONENT_ALL				0xffffffff

#define VXGE_HW_BASE_INF	100
#define VXGE_HW_BASE_ERR	200
#define VXGE_HW_BASE_BADCFG	300

enum vxge_hw_status {
	VXGE_HW_OK				  = 0,
	VXGE_HW_FAIL				  = 1,
	VXGE_HW_PENDING				  = 2,
	VXGE_HW_COMPLETIONS_REMAIN		  = 3,

	VXGE_HW_INF_NO_MORE_COMPLETED_DESCRIPTORS = VXGE_HW_BASE_INF + 1,
	VXGE_HW_INF_OUT_OF_DESCRIPTORS		  = VXGE_HW_BASE_INF + 2,

	VXGE_HW_ERR_INVALID_HANDLE		  = VXGE_HW_BASE_ERR + 1,
	VXGE_HW_ERR_OUT_OF_MEMORY		  = VXGE_HW_BASE_ERR + 2,
	VXGE_HW_ERR_VPATH_NOT_AVAILABLE	  	  = VXGE_HW_BASE_ERR + 3,
	VXGE_HW_ERR_VPATH_NOT_OPEN		  = VXGE_HW_BASE_ERR + 4,
	VXGE_HW_ERR_WRONG_IRQ			  = VXGE_HW_BASE_ERR + 5,
	VXGE_HW_ERR_SWAPPER_CTRL		  = VXGE_HW_BASE_ERR + 6,
	VXGE_HW_ERR_INVALID_MTU_SIZE		  = VXGE_HW_BASE_ERR + 7,
	VXGE_HW_ERR_INVALID_INDEX		  = VXGE_HW_BASE_ERR + 8,
	VXGE_HW_ERR_INVALID_TYPE		  = VXGE_HW_BASE_ERR + 9,
	VXGE_HW_ERR_INVALID_OFFSET		  = VXGE_HW_BASE_ERR + 10,
	VXGE_HW_ERR_INVALID_DEVICE		  = VXGE_HW_BASE_ERR + 11,
	VXGE_HW_ERR_VERSION_CONFLICT		  = VXGE_HW_BASE_ERR + 12,
	VXGE_HW_ERR_INVALID_PCI_INFO		  = VXGE_HW_BASE_ERR + 13,
	VXGE_HW_ERR_INVALID_TCODE 		  = VXGE_HW_BASE_ERR + 14,
	VXGE_HW_ERR_INVALID_BLOCK_SIZE		  = VXGE_HW_BASE_ERR + 15,
	VXGE_HW_ERR_INVALID_STATE		  = VXGE_HW_BASE_ERR + 16,
	VXGE_HW_ERR_PRIVILAGED_OPEARATION	  = VXGE_HW_BASE_ERR + 17,
	VXGE_HW_ERR_INVALID_PORT 		  = VXGE_HW_BASE_ERR + 18,
	VXGE_HW_ERR_FIFO		 	  = VXGE_HW_BASE_ERR + 19,
	VXGE_HW_ERR_VPATH			  = VXGE_HW_BASE_ERR + 20,
	VXGE_HW_ERR_CRITICAL			  = VXGE_HW_BASE_ERR + 21,
	VXGE_HW_ERR_SLOT_FREEZE 		  = VXGE_HW_BASE_ERR + 22,

	VXGE_HW_BADCFG_RING_INDICATE_MAX_PKTS	  = VXGE_HW_BASE_BADCFG + 1,
	VXGE_HW_BADCFG_FIFO_BLOCKS		  = VXGE_HW_BASE_BADCFG + 2,
	VXGE_HW_BADCFG_VPATH_MTU		  = VXGE_HW_BASE_BADCFG + 3,
	VXGE_HW_BADCFG_VPATH_RPA_STRIP_VLAN_TAG	  = VXGE_HW_BASE_BADCFG + 4,
	VXGE_HW_BADCFG_VPATH_MIN_BANDWIDTH	  = VXGE_HW_BASE_BADCFG + 5,
	VXGE_HW_BADCFG_INTR_MODE		  = VXGE_HW_BASE_BADCFG + 6,
	VXGE_HW_BADCFG_RTS_MAC_EN		  = VXGE_HW_BASE_BADCFG + 7,

	VXGE_HW_EOF_TRACE_BUF			  = -1
};

enum vxge_hw_device_link_state {
	VXGE_HW_LINK_NONE,
	VXGE_HW_LINK_DOWN,
	VXGE_HW_LINK_UP
};


#define VXGE_HW_FW_STRLEN	32
struct vxge_hw_device_date {
	u32     day;
	u32     month;
	u32     year;
	char    date[VXGE_HW_FW_STRLEN];
};

struct vxge_hw_device_version {
	u32     major;
	u32     minor;
	u32     build;
	char    version[VXGE_HW_FW_STRLEN];
};

u64
__vxge_hw_vpath_pci_func_mode_get(
	u32 vp_id,
	struct vxge_hw_vpath_reg __iomem *vpath_reg);

struct vxge_hw_fifo_config {
	u32				enable;
#define VXGE_HW_FIFO_ENABLE				1
#define VXGE_HW_FIFO_DISABLE				0

	u32				fifo_blocks;
#define VXGE_HW_MIN_FIFO_BLOCKS				2
#define VXGE_HW_MAX_FIFO_BLOCKS				128

	u32				max_frags;
#define VXGE_HW_MIN_FIFO_FRAGS				1
#define VXGE_HW_MAX_FIFO_FRAGS				256

	u32				memblock_size;
#define VXGE_HW_MIN_FIFO_MEMBLOCK_SIZE			VXGE_HW_BLOCK_SIZE
#define VXGE_HW_MAX_FIFO_MEMBLOCK_SIZE			131072
#define VXGE_HW_DEF_FIFO_MEMBLOCK_SIZE			8096

	u32		                alignment_size;
#define VXGE_HW_MIN_FIFO_ALIGNMENT_SIZE		0
#define VXGE_HW_MAX_FIFO_ALIGNMENT_SIZE		65536
#define VXGE_HW_DEF_FIFO_ALIGNMENT_SIZE		VXGE_CACHE_LINE_SIZE

	u32		                intr;
#define VXGE_HW_FIFO_QUEUE_INTR_ENABLE			1
#define VXGE_HW_FIFO_QUEUE_INTR_DISABLE			0
#define VXGE_HW_FIFO_QUEUE_INTR_DEFAULT			0

	u32				no_snoop_bits;
#define VXGE_HW_FIFO_NO_SNOOP_DISABLED			0
#define VXGE_HW_FIFO_NO_SNOOP_TXD			1
#define VXGE_HW_FIFO_NO_SNOOP_FRM			2
#define VXGE_HW_FIFO_NO_SNOOP_ALL			3
#define VXGE_HW_FIFO_NO_SNOOP_DEFAULT			0

};
struct vxge_hw_ring_config {
	u32				enable;
#define VXGE_HW_RING_ENABLE					1
#define VXGE_HW_RING_DISABLE					0
#define VXGE_HW_RING_DEFAULT					1

	u32				ring_blocks;
#define VXGE_HW_MIN_RING_BLOCKS				1
#define VXGE_HW_MAX_RING_BLOCKS				128
#define VXGE_HW_DEF_RING_BLOCKS				2

	u32				buffer_mode;
#define VXGE_HW_RING_RXD_BUFFER_MODE_1				1
#define VXGE_HW_RING_RXD_BUFFER_MODE_3				3
#define VXGE_HW_RING_RXD_BUFFER_MODE_5				5
#define VXGE_HW_RING_RXD_BUFFER_MODE_DEFAULT			1

	u32				scatter_mode;
#define VXGE_HW_RING_SCATTER_MODE_A				0
#define VXGE_HW_RING_SCATTER_MODE_B				1
#define VXGE_HW_RING_SCATTER_MODE_C				2
#define VXGE_HW_RING_SCATTER_MODE_USE_FLASH_DEFAULT		0xffffffff

	u64				rxds_limit;
#define VXGE_HW_DEF_RING_RXDS_LIMIT				44
};

struct vxge_hw_vp_config {
	u32				vp_id;

#define	VXGE_HW_VPATH_PRIORITY_MIN			0
#define	VXGE_HW_VPATH_PRIORITY_MAX			16
#define	VXGE_HW_VPATH_PRIORITY_DEFAULT			0

	u32				min_bandwidth;
#define	VXGE_HW_VPATH_BANDWIDTH_MIN			0
#define	VXGE_HW_VPATH_BANDWIDTH_MAX			100
#define	VXGE_HW_VPATH_BANDWIDTH_DEFAULT			0

	struct vxge_hw_ring_config		ring;
	struct vxge_hw_fifo_config		fifo;
	struct vxge_hw_tim_intr_config	tti;
	struct vxge_hw_tim_intr_config	rti;

	u32				mtu;
#define VXGE_HW_VPATH_MIN_INITIAL_MTU			VXGE_HW_MIN_MTU
#define VXGE_HW_VPATH_MAX_INITIAL_MTU			VXGE_HW_MAX_MTU
#define VXGE_HW_VPATH_USE_FLASH_DEFAULT_INITIAL_MTU	0xffffffff

	u32				rpa_strip_vlan_tag;
#define VXGE_HW_VPATH_RPA_STRIP_VLAN_TAG_ENABLE			1
#define VXGE_HW_VPATH_RPA_STRIP_VLAN_TAG_DISABLE		0
#define VXGE_HW_VPATH_RPA_STRIP_VLAN_TAG_USE_FLASH_DEFAULT	0xffffffff

};
struct vxge_hw_device_config {
	u32				dma_blockpool_initial;
	u32				dma_blockpool_max;
#define VXGE_HW_MIN_DMA_BLOCK_POOL_SIZE			0
#define VXGE_HW_INITIAL_DMA_BLOCK_POOL_SIZE		0
#define VXGE_HW_INCR_DMA_BLOCK_POOL_SIZE		4
#define VXGE_HW_MAX_DMA_BLOCK_POOL_SIZE			4096

#define        VXGE_HW_MAX_PAYLOAD_SIZE_512		2

	u32				intr_mode;
#define VXGE_HW_INTR_MODE_IRQLINE			0
#define VXGE_HW_INTR_MODE_MSIX				1
#define VXGE_HW_INTR_MODE_MSIX_ONE_SHOT			2

#define VXGE_HW_INTR_MODE_DEF				0

	u32				rth_en;
#define VXGE_HW_RTH_DISABLE				0
#define VXGE_HW_RTH_ENABLE				1
#define VXGE_HW_RTH_DEFAULT				0

	u32				rth_it_type;
#define VXGE_HW_RTH_IT_TYPE_SOLO_IT			0
#define VXGE_HW_RTH_IT_TYPE_MULTI_IT			1
#define VXGE_HW_RTH_IT_TYPE_DEFAULT			0

	u32				rts_mac_en;
#define VXGE_HW_RTS_MAC_DISABLE			0
#define VXGE_HW_RTS_MAC_ENABLE			1
#define VXGE_HW_RTS_MAC_DEFAULT			0

	struct vxge_hw_vp_config	vp_config[VXGE_HW_MAX_VIRTUAL_PATHS];

	u32				device_poll_millis;
#define VXGE_HW_MIN_DEVICE_POLL_MILLIS			1
#define VXGE_HW_MAX_DEVICE_POLL_MILLIS			100000
#define VXGE_HW_DEF_DEVICE_POLL_MILLIS			1000

};




struct vxge_hw_uld_cbs {

	void (*link_up)(struct __vxge_hw_device *devh);
	void (*link_down)(struct __vxge_hw_device *devh);
	void (*crit_err)(struct __vxge_hw_device *devh,
			enum vxge_hw_event type, u64 ext_data);
};

struct __vxge_hw_blockpool_entry {
	struct list_head	item;
	u32			length;
	void			*memblock;
	dma_addr_t		dma_addr;
	struct pci_dev 		*dma_handle;
	struct pci_dev 		*acc_handle;
};

struct __vxge_hw_blockpool {
	struct __vxge_hw_device *hldev;
	u32				block_size;
	u32				pool_size;
	u32				pool_max;
	u32				req_out;
	struct list_head		free_block_list;
	struct list_head		free_entry_list;
};

enum __vxge_hw_channel_type {
	VXGE_HW_CHANNEL_TYPE_UNKNOWN			= 0,
	VXGE_HW_CHANNEL_TYPE_FIFO			= 1,
	VXGE_HW_CHANNEL_TYPE_RING			= 2,
	VXGE_HW_CHANNEL_TYPE_MAX			= 3
};

struct __vxge_hw_channel {
	struct list_head		item;
	enum __vxge_hw_channel_type	type;
	struct __vxge_hw_device 	*devh;
	struct __vxge_hw_vpath_handle 	*vph;
	u32			length;
	u32			vp_id;
	void		**reserve_arr;
	u32			reserve_ptr;
	u32			reserve_top;
	void		**work_arr;
	u32			post_index ____cacheline_aligned;
	u32			compl_index ____cacheline_aligned;
	void		**free_arr;
	u32			free_ptr;
	void		**orig_arr;
	u32			per_dtr_space;
	void		*userdata;
	struct vxge_hw_common_reg	__iomem *common_reg;
	u32			first_vp_id;
	struct vxge_hw_vpath_stats_sw_common_info *stats;

} ____cacheline_aligned;

struct __vxge_hw_virtualpath {
	u32				vp_id;

	u32				vp_open;
#define VXGE_HW_VP_NOT_OPEN	0
#define	VXGE_HW_VP_OPEN		1

	struct __vxge_hw_device		*hldev;
	struct vxge_hw_vp_config	*vp_config;
	struct vxge_hw_vpath_reg	__iomem *vp_reg;
	struct vxge_hw_vpmgmt_reg	__iomem *vpmgmt_reg;
	struct __vxge_hw_non_offload_db_wrapper	__iomem *nofl_db;

	u32				max_mtu;
	u32				vsport_number;
	u32				max_kdfc_db;
	u32				max_nofl_db;

	struct __vxge_hw_ring *____cacheline_aligned ringh;
	struct __vxge_hw_fifo *____cacheline_aligned fifoh;
	struct list_head		vpath_handles;
	struct __vxge_hw_blockpool_entry		*stats_block;
	struct vxge_hw_vpath_stats_hw_info	*hw_stats;
	struct vxge_hw_vpath_stats_hw_info	*hw_stats_sav;
	struct vxge_hw_vpath_stats_sw_info	*sw_stats;
};

struct __vxge_hw_vpath_handle{
	struct list_head	item;
	struct __vxge_hw_virtualpath	*vpath;
};

struct __vxge_hw_device {
	u32				magic;
#define VXGE_HW_DEVICE_MAGIC		0x12345678
#define VXGE_HW_DEVICE_DEAD		0xDEADDEAD
	u16				device_id;
	u8				major_revision;
	u8				minor_revision;
	void __iomem			*bar0;
	struct pci_dev			*pdev;
	struct net_device		*ndev;
	struct vxge_hw_device_config	config;
	enum vxge_hw_device_link_state	link_state;

	struct vxge_hw_uld_cbs		uld_callbacks;

	u32				host_type;
	u32				func_id;
	u32				access_rights;
#define VXGE_HW_DEVICE_ACCESS_RIGHT_VPATH      0x1
#define VXGE_HW_DEVICE_ACCESS_RIGHT_SRPCIM     0x2
#define VXGE_HW_DEVICE_ACCESS_RIGHT_MRPCIM     0x4
	struct vxge_hw_legacy_reg	__iomem *legacy_reg;
	struct vxge_hw_toc_reg		__iomem *toc_reg;
	struct vxge_hw_common_reg	__iomem *common_reg;
	struct vxge_hw_mrpcim_reg	__iomem *mrpcim_reg;
	struct vxge_hw_srpcim_reg	__iomem *srpcim_reg \
					[VXGE_HW_TITAN_SRPCIM_REG_SPACES];
	struct vxge_hw_vpmgmt_reg	__iomem *vpmgmt_reg \
					[VXGE_HW_TITAN_VPMGMT_REG_SPACES];
	struct vxge_hw_vpath_reg	__iomem *vpath_reg \
					[VXGE_HW_TITAN_VPATH_REG_SPACES];
	u8				__iomem *kdfc;
	u8				__iomem *usdc;
	struct __vxge_hw_virtualpath	virtual_paths \
					[VXGE_HW_MAX_VIRTUAL_PATHS];
	u64				vpath_assignments;
	u64				vpaths_deployed;
	u32				first_vp_id;
	u64				tim_int_mask0[4];
	u32				tim_int_mask1[4];

	struct __vxge_hw_blockpool	block_pool;
	struct vxge_hw_device_stats	stats;
	u32				debug_module_mask;
	u32				debug_level;
	u32				level_err;
	u32				level_trace;
};

#define VXGE_HW_INFO_LEN	64
struct vxge_hw_device_hw_info {
	u32		host_type;
#define VXGE_HW_NO_MR_NO_SR_NORMAL_FUNCTION			0
#define VXGE_HW_MR_NO_SR_VH0_BASE_FUNCTION			1
#define VXGE_HW_NO_MR_SR_VH0_FUNCTION0				2
#define VXGE_HW_NO_MR_SR_VH0_VIRTUAL_FUNCTION			3
#define VXGE_HW_MR_SR_VH0_INVALID_CONFIG			4
#define VXGE_HW_SR_VH_FUNCTION0					5
#define VXGE_HW_SR_VH_VIRTUAL_FUNCTION				6
#define VXGE_HW_VH_NORMAL_FUNCTION				7
	u64		function_mode;
#define VXGE_HW_FUNCTION_MODE_SINGLE_FUNCTION			0
#define VXGE_HW_FUNCTION_MODE_MULTI_FUNCTION			1
#define VXGE_HW_FUNCTION_MODE_SRIOV				2
#define VXGE_HW_FUNCTION_MODE_MRIOV				3
#define VXGE_HW_FUNCTION_MODE_MRIOV_8				4
#define VXGE_HW_FUNCTION_MODE_MULTI_FUNCTION_17			5
#define VXGE_HW_FUNCTION_MODE_SRIOV_8				6
#define VXGE_HW_FUNCTION_MODE_SRIOV_4				7
#define VXGE_HW_FUNCTION_MODE_MULTI_FUNCTION_2			8
#define VXGE_HW_FUNCTION_MODE_MULTI_FUNCTION_4			9
#define VXGE_HW_FUNCTION_MODE_MRIOV_4				10

	u32		func_id;
	u64		vpath_mask;
	struct vxge_hw_device_version fw_version;
	struct vxge_hw_device_date    fw_date;
	struct vxge_hw_device_version flash_version;
	struct vxge_hw_device_date    flash_date;
	u8		serial_number[VXGE_HW_INFO_LEN];
	u8		part_number[VXGE_HW_INFO_LEN];
	u8		product_desc[VXGE_HW_INFO_LEN];
	u8 (mac_addrs)[VXGE_HW_MAX_VIRTUAL_PATHS][ETH_ALEN];
	u8 (mac_addr_masks)[VXGE_HW_MAX_VIRTUAL_PATHS][ETH_ALEN];
};

struct vxge_hw_device_attr {
	void __iomem		*bar0;
	struct pci_dev 		*pdev;
	struct vxge_hw_uld_cbs	uld_callbacks;
};

#define VXGE_HW_DEVICE_LINK_STATE_SET(hldev, ls)	(hldev->link_state = ls)

#define VXGE_HW_DEVICE_TIM_INT_MASK_SET(m0, m1, i) {	\
	if (i < 16) {				\
		m0[0] |= vxge_vBIT(0x8, (i*4), 4);	\
		m0[1] |= vxge_vBIT(0x4, (i*4), 4);	\
	}			       		\
	else {					\
		m1[0] = 0x80000000;		\
		m1[1] = 0x40000000;		\
	}					\
}

#define VXGE_HW_DEVICE_TIM_INT_MASK_RESET(m0, m1, i) {	\
	if (i < 16) {					\
		m0[0] &= ~vxge_vBIT(0x8, (i*4), 4);		\
		m0[1] &= ~vxge_vBIT(0x4, (i*4), 4);		\
	}						\
	else {						\
		m1[0] = 0;				\
		m1[1] = 0;				\
	}						\
}

#define VXGE_HW_DEVICE_STATS_PIO_READ(loc, offset) {		\
	status = vxge_hw_mrpcim_stats_access(hldev, \
				VXGE_HW_STATS_OP_READ, \
				loc, \
				offset, \
				&val64);			\
								\
	if (status != VXGE_HW_OK)				\
		return status;						\
}

#define VXGE_HW_VPATH_STATS_PIO_READ(offset) {				\
	status = __vxge_hw_vpath_stats_access(vpath, \
			VXGE_HW_STATS_OP_READ, \
			offset, \
			&val64);					\
	if (status != VXGE_HW_OK)					\
		return status;						\
}

struct __vxge_hw_ring {
	struct __vxge_hw_channel		channel;
	struct vxge_hw_mempool			*mempool;
	struct vxge_hw_vpath_reg		__iomem	*vp_reg;
	struct vxge_hw_common_reg		__iomem	*common_reg;
	u32					ring_length;
	u32					buffer_mode;
	u32					rxd_size;
	u32					rxd_priv_size;
	u32					per_rxd_space;
	u32					rxds_per_block;
	u32					rxdblock_priv_size;
	u32					cmpl_cnt;
	u32					vp_id;
	u32					doorbell_cnt;
	u32					total_db_cnt;
	u64					rxds_limit;

	enum vxge_hw_status (*callback)(
			struct __vxge_hw_ring *ringh,
			void *rxdh,
			u8 t_code,
			void *userdata);

	enum vxge_hw_status (*rxd_init)(
			void *rxdh,
			void *userdata);

	void (*rxd_term)(
			void *rxdh,
			enum vxge_hw_rxd_state state,
			void *userdata);

	struct vxge_hw_vpath_stats_sw_ring_info *stats	____cacheline_aligned;
	struct vxge_hw_ring_config		*config;
} ____cacheline_aligned;

enum vxge_hw_txdl_state {
	VXGE_HW_TXDL_STATE_NONE	= 0,
	VXGE_HW_TXDL_STATE_AVAIL	= 1,
	VXGE_HW_TXDL_STATE_POSTED	= 2,
	VXGE_HW_TXDL_STATE_FREED	= 3
};
struct __vxge_hw_fifo {
	struct __vxge_hw_channel		channel;
	struct vxge_hw_mempool			*mempool;
	struct vxge_hw_fifo_config		*config;
	struct vxge_hw_vpath_reg		__iomem *vp_reg;
	struct __vxge_hw_non_offload_db_wrapper	__iomem *nofl_db;
	u64					interrupt_type;
	u32					no_snoop_bits;
	u32					txdl_per_memblock;
	u32					txdl_size;
	u32					priv_size;
	u32					per_txdl_space;
	u32					vp_id;
	u32					tx_intr_num;

	enum vxge_hw_status (*callback)(
			struct __vxge_hw_fifo *fifo_handle,
			void *txdlh,
			enum vxge_hw_fifo_tcode t_code,
			void *userdata,
			struct sk_buff ***skb_ptr,
			int nr_skb,
			int *more);

	void (*txdl_term)(
			void *txdlh,
			enum vxge_hw_txdl_state state,
			void *userdata);

	struct vxge_hw_vpath_stats_sw_fifo_info *stats ____cacheline_aligned;
} ____cacheline_aligned;

struct __vxge_hw_fifo_txdl_priv {
	dma_addr_t		dma_addr;
	struct pci_dev	*dma_handle;
	ptrdiff_t		dma_offset;
	u32				frags;
	u8				*align_vaddr_start;
	u8				*align_vaddr;
	dma_addr_t		align_dma_addr;
	struct pci_dev 	*align_dma_handle;
	struct pci_dev	*align_dma_acch;
	ptrdiff_t		align_dma_offset;
	u32				align_used_frags;
	u32				alloc_frags;
	u32				unused;
	struct __vxge_hw_fifo_txdl_priv	*next_txdl_priv;
	struct vxge_hw_fifo_txd		*first_txdp;
	void			*memblock;
};

struct __vxge_hw_non_offload_db_wrapper {
	u64		control_0;
#define	VXGE_HW_NODBW_GET_TYPE(ctrl0)			vxge_bVALn(ctrl0, 0, 8)
#define VXGE_HW_NODBW_TYPE(val) vxge_vBIT(val, 0, 8)
#define	VXGE_HW_NODBW_TYPE_NODBW				0

#define	VXGE_HW_NODBW_GET_LAST_TXD_NUMBER(ctrl0)	vxge_bVALn(ctrl0, 32, 8)
#define VXGE_HW_NODBW_LAST_TXD_NUMBER(val) vxge_vBIT(val, 32, 8)

#define	VXGE_HW_NODBW_GET_NO_SNOOP(ctrl0)		vxge_bVALn(ctrl0, 56, 8)
#define VXGE_HW_NODBW_LIST_NO_SNOOP(val) vxge_vBIT(val, 56, 8)
#define	VXGE_HW_NODBW_LIST_NO_SNOOP_TXD_READ_TXD0_WRITE		0x2
#define	VXGE_HW_NODBW_LIST_NO_SNOOP_TX_FRAME_DATA_READ		0x1

	u64		txdl_ptr;
};


struct vxge_hw_fifo_txd {
	u64 control_0;
#define VXGE_HW_FIFO_TXD_LIST_OWN_ADAPTER		vxge_mBIT(7)

#define VXGE_HW_FIFO_TXD_T_CODE_GET(ctrl0)		vxge_bVALn(ctrl0, 12, 4)
#define VXGE_HW_FIFO_TXD_T_CODE(val) 			vxge_vBIT(val, 12, 4)
#define VXGE_HW_FIFO_TXD_T_CODE_UNUSED		VXGE_HW_FIFO_T_CODE_UNUSED


#define VXGE_HW_FIFO_TXD_GATHER_CODE(val) 		vxge_vBIT(val, 22, 2)
#define VXGE_HW_FIFO_TXD_GATHER_CODE_FIRST	VXGE_HW_FIFO_GATHER_CODE_FIRST
#define VXGE_HW_FIFO_TXD_GATHER_CODE_LAST	VXGE_HW_FIFO_GATHER_CODE_LAST


#define VXGE_HW_FIFO_TXD_LSO_EN				vxge_mBIT(30)

#define VXGE_HW_FIFO_TXD_LSO_MSS(val) 			vxge_vBIT(val, 34, 14)

#define VXGE_HW_FIFO_TXD_BUFFER_SIZE(val) 		vxge_vBIT(val, 48, 16)

	u64 control_1;
#define VXGE_HW_FIFO_TXD_TX_CKO_IPV4_EN			vxge_mBIT(5)
#define VXGE_HW_FIFO_TXD_TX_CKO_TCP_EN			vxge_mBIT(6)
#define VXGE_HW_FIFO_TXD_TX_CKO_UDP_EN			vxge_mBIT(7)
#define VXGE_HW_FIFO_TXD_VLAN_ENABLE			vxge_mBIT(15)

#define VXGE_HW_FIFO_TXD_VLAN_TAG(val) 			vxge_vBIT(val, 16, 16)

#define VXGE_HW_FIFO_TXD_INT_NUMBER(val) 		vxge_vBIT(val, 34, 6)

#define VXGE_HW_FIFO_TXD_INT_TYPE_PER_LIST		vxge_mBIT(46)
#define VXGE_HW_FIFO_TXD_INT_TYPE_UTILZ			vxge_mBIT(47)

	u64 buffer_pointer;

	u64 host_control;
};

struct vxge_hw_ring_rxd_1 {
	u64 host_control;
	u64 control_0;
#define VXGE_HW_RING_RXD_RTH_BUCKET_GET(ctrl0)		vxge_bVALn(ctrl0, 0, 7)

#define VXGE_HW_RING_RXD_LIST_OWN_ADAPTER		vxge_mBIT(7)

#define VXGE_HW_RING_RXD_FAST_PATH_ELIGIBLE_GET(ctrl0)	vxge_bVALn(ctrl0, 8, 1)

#define VXGE_HW_RING_RXD_L3_CKSUM_CORRECT_GET(ctrl0)	vxge_bVALn(ctrl0, 9, 1)

#define VXGE_HW_RING_RXD_L4_CKSUM_CORRECT_GET(ctrl0)	vxge_bVALn(ctrl0, 10, 1)

#define VXGE_HW_RING_RXD_T_CODE_GET(ctrl0)		vxge_bVALn(ctrl0, 12, 4)
#define VXGE_HW_RING_RXD_T_CODE(val) 			vxge_vBIT(val, 12, 4)

#define VXGE_HW_RING_RXD_T_CODE_UNUSED		VXGE_HW_RING_T_CODE_UNUSED

#define VXGE_HW_RING_RXD_SYN_GET(ctrl0)		vxge_bVALn(ctrl0, 16, 1)

#define VXGE_HW_RING_RXD_IS_ICMP_GET(ctrl0)		vxge_bVALn(ctrl0, 17, 1)

#define VXGE_HW_RING_RXD_RTH_SPDM_HIT_GET(ctrl0)	vxge_bVALn(ctrl0, 18, 1)

#define VXGE_HW_RING_RXD_RTH_IT_HIT_GET(ctrl0)		vxge_bVALn(ctrl0, 19, 1)

#define VXGE_HW_RING_RXD_RTH_HASH_TYPE_GET(ctrl0)	vxge_bVALn(ctrl0, 20, 4)

#define VXGE_HW_RING_RXD_IS_VLAN_GET(ctrl0)		vxge_bVALn(ctrl0, 24, 1)

#define VXGE_HW_RING_RXD_ETHER_ENCAP_GET(ctrl0)		vxge_bVALn(ctrl0, 25, 2)

#define VXGE_HW_RING_RXD_FRAME_PROTO_GET(ctrl0)		vxge_bVALn(ctrl0, 27, 5)

#define VXGE_HW_RING_RXD_L3_CKSUM_GET(ctrl0)	vxge_bVALn(ctrl0, 32, 16)

#define VXGE_HW_RING_RXD_L4_CKSUM_GET(ctrl0)	vxge_bVALn(ctrl0, 48, 16)

	u64 control_1;

#define VXGE_HW_RING_RXD_1_BUFFER0_SIZE_GET(ctrl1)	vxge_bVALn(ctrl1, 2, 14)
#define VXGE_HW_RING_RXD_1_BUFFER0_SIZE(val) vxge_vBIT(val, 2, 14)
#define VXGE_HW_RING_RXD_1_BUFFER0_SIZE_MASK		vxge_vBIT(0x3FFF, 2, 14)

#define VXGE_HW_RING_RXD_1_RTH_HASH_VAL_GET(ctrl1)    vxge_bVALn(ctrl1, 16, 32)

#define VXGE_HW_RING_RXD_VLAN_TAG_GET(ctrl1)	vxge_bVALn(ctrl1, 48, 16)

	u64 buffer0_ptr;
};

enum vxge_hw_rth_algoritms {
	RTH_ALG_JENKINS = 0,
	RTH_ALG_MS_RSS	= 1,
	RTH_ALG_CRC32C	= 2
};

struct vxge_hw_rth_hash_types {
	u8 hash_type_tcpipv4_en;
	u8 hash_type_ipv4_en;
	u8 hash_type_tcpipv6_en;
	u8 hash_type_ipv6_en;
	u8 hash_type_tcpipv6ex_en;
	u8 hash_type_ipv6ex_en;
};

u32
vxge_hw_device_debug_mask_get(struct __vxge_hw_device *devh);

void vxge_hw_device_debug_set(
	struct __vxge_hw_device *devh,
	enum vxge_debug_level level,
	u32 mask);

u32
vxge_hw_device_error_level_get(struct __vxge_hw_device *devh);

u32
vxge_hw_device_trace_level_get(struct __vxge_hw_device *devh);

u32
vxge_hw_device_debug_mask_get(struct __vxge_hw_device *devh);

static inline u32 vxge_hw_ring_rxd_size_get(u32 buf_mode)
{
	return sizeof(struct vxge_hw_ring_rxd_1);
}

static inline u32 vxge_hw_ring_rxds_per_block_get(u32 buf_mode)
{
	return (u32)((VXGE_HW_BLOCK_SIZE-16) /
		sizeof(struct vxge_hw_ring_rxd_1));
}

static inline
void vxge_hw_ring_rxd_1b_set(
	void *rxdh,
	dma_addr_t dma_pointer,
	u32 size)
{
	struct vxge_hw_ring_rxd_1 *rxdp = (struct vxge_hw_ring_rxd_1 *)rxdh;
	rxdp->buffer0_ptr = dma_pointer;
	rxdp->control_1	&= ~VXGE_HW_RING_RXD_1_BUFFER0_SIZE_MASK;
	rxdp->control_1	|= VXGE_HW_RING_RXD_1_BUFFER0_SIZE(size);
}

static inline
void vxge_hw_ring_rxd_1b_get(
	struct __vxge_hw_ring *ring_handle,
	void *rxdh,
	u32 *pkt_length)
{
	struct vxge_hw_ring_rxd_1 *rxdp = (struct vxge_hw_ring_rxd_1 *)rxdh;

	*pkt_length =
		(u32)VXGE_HW_RING_RXD_1_BUFFER0_SIZE_GET(rxdp->control_1);
}

static inline
void vxge_hw_ring_rxd_1b_info_get(
	struct __vxge_hw_ring *ring_handle,
	void *rxdh,
	struct vxge_hw_ring_rxd_info *rxd_info)
{

	struct vxge_hw_ring_rxd_1 *rxdp = (struct vxge_hw_ring_rxd_1 *)rxdh;
	rxd_info->syn_flag =
		(u32)VXGE_HW_RING_RXD_SYN_GET(rxdp->control_0);
	rxd_info->is_icmp =
		(u32)VXGE_HW_RING_RXD_IS_ICMP_GET(rxdp->control_0);
	rxd_info->fast_path_eligible =
		(u32)VXGE_HW_RING_RXD_FAST_PATH_ELIGIBLE_GET(rxdp->control_0);
	rxd_info->l3_cksum_valid =
		(u32)VXGE_HW_RING_RXD_L3_CKSUM_CORRECT_GET(rxdp->control_0);
	rxd_info->l3_cksum =
		(u32)VXGE_HW_RING_RXD_L3_CKSUM_GET(rxdp->control_0);
	rxd_info->l4_cksum_valid =
		(u32)VXGE_HW_RING_RXD_L4_CKSUM_CORRECT_GET(rxdp->control_0);
	rxd_info->l4_cksum =
		(u32)VXGE_HW_RING_RXD_L4_CKSUM_GET(rxdp->control_0);
	rxd_info->frame =
		(u32)VXGE_HW_RING_RXD_ETHER_ENCAP_GET(rxdp->control_0);
	rxd_info->proto =
		(u32)VXGE_HW_RING_RXD_FRAME_PROTO_GET(rxdp->control_0);
	rxd_info->is_vlan =
		(u32)VXGE_HW_RING_RXD_IS_VLAN_GET(rxdp->control_0);
	rxd_info->vlan =
		(u32)VXGE_HW_RING_RXD_VLAN_TAG_GET(rxdp->control_1);
	rxd_info->rth_bucket =
		(u32)VXGE_HW_RING_RXD_RTH_BUCKET_GET(rxdp->control_0);
	rxd_info->rth_it_hit =
		(u32)VXGE_HW_RING_RXD_RTH_IT_HIT_GET(rxdp->control_0);
	rxd_info->rth_spdm_hit =
		(u32)VXGE_HW_RING_RXD_RTH_SPDM_HIT_GET(rxdp->control_0);
	rxd_info->rth_hash_type =
		(u32)VXGE_HW_RING_RXD_RTH_HASH_TYPE_GET(rxdp->control_0);
	rxd_info->rth_value =
		(u32)VXGE_HW_RING_RXD_1_RTH_HASH_VAL_GET(rxdp->control_1);
}

static inline void *vxge_hw_ring_rxd_private_get(void *rxdh)
{
	struct vxge_hw_ring_rxd_1 *rxdp = (struct vxge_hw_ring_rxd_1 *)rxdh;
	return (void *)(size_t)rxdp->host_control;
}

static inline void vxge_hw_fifo_txdl_cksum_set_bits(void *txdlh, u64 cksum_bits)
{
	struct vxge_hw_fifo_txd *txdp = (struct vxge_hw_fifo_txd *)txdlh;
	txdp->control_1 |= cksum_bits;
}

static inline void vxge_hw_fifo_txdl_mss_set(void *txdlh, int mss)
{
	struct vxge_hw_fifo_txd *txdp = (struct vxge_hw_fifo_txd *)txdlh;

	txdp->control_0 |= VXGE_HW_FIFO_TXD_LSO_EN;
	txdp->control_0 |= VXGE_HW_FIFO_TXD_LSO_MSS(mss);
}

static inline void vxge_hw_fifo_txdl_vlan_set(void *txdlh, u16 vlan_tag)
{
	struct vxge_hw_fifo_txd *txdp = (struct vxge_hw_fifo_txd *)txdlh;

	txdp->control_1 |= VXGE_HW_FIFO_TXD_VLAN_ENABLE;
	txdp->control_1 |= VXGE_HW_FIFO_TXD_VLAN_TAG(vlan_tag);
}

static inline void *vxge_hw_fifo_txdl_private_get(void *txdlh)
{
	struct vxge_hw_fifo_txd *txdp  = (struct vxge_hw_fifo_txd *)txdlh;

	return (void *)(size_t)txdp->host_control;
}

struct vxge_hw_ring_attr {
	enum vxge_hw_status (*callback)(
			struct __vxge_hw_ring *ringh,
			void *rxdh,
			u8 t_code,
			void *userdata);

	enum vxge_hw_status (*rxd_init)(
			void *rxdh,
			void *userdata);

	void (*rxd_term)(
			void *rxdh,
			enum vxge_hw_rxd_state state,
			void *userdata);

	void		*userdata;
	u32		per_rxd_space;
};

struct vxge_hw_fifo_attr {

	enum vxge_hw_status (*callback)(
			struct __vxge_hw_fifo *fifo_handle,
			void *txdlh,
			enum vxge_hw_fifo_tcode t_code,
			void *userdata,
			struct sk_buff ***skb_ptr,
			int nr_skb, int *more);

	void (*txdl_term)(
			void *txdlh,
			enum vxge_hw_txdl_state state,
			void *userdata);

	void		*userdata;
	u32		per_txdl_space;
};

struct vxge_hw_vpath_attr {
	u32				vp_id;
	struct vxge_hw_ring_attr	ring_attr;
	struct vxge_hw_fifo_attr	fifo_attr;
};

enum vxge_hw_status
__vxge_hw_blockpool_create(struct __vxge_hw_device *hldev,
			struct __vxge_hw_blockpool  *blockpool,
			u32 pool_size,
			u32 pool_max);

void
__vxge_hw_blockpool_destroy(struct __vxge_hw_blockpool  *blockpool);

struct __vxge_hw_blockpool_entry *
__vxge_hw_blockpool_block_allocate(struct __vxge_hw_device *hldev,
			u32 size);

void
__vxge_hw_blockpool_block_free(struct __vxge_hw_device *hldev,
			struct __vxge_hw_blockpool_entry *entry);

void *
__vxge_hw_blockpool_malloc(struct __vxge_hw_device *hldev,
			u32 size,
			struct vxge_hw_mempool_dma *dma_object);

void
__vxge_hw_blockpool_free(struct __vxge_hw_device *hldev,
			void *memblock,
			u32 size,
			struct vxge_hw_mempool_dma *dma_object);

enum vxge_hw_status
__vxge_hw_device_fifo_config_check(struct vxge_hw_fifo_config *fifo_config);

enum vxge_hw_status
__vxge_hw_device_config_check(struct vxge_hw_device_config *new_config);

enum vxge_hw_status
vxge_hw_mgmt_device_config(struct __vxge_hw_device *devh,
		struct vxge_hw_device_config	*dev_config, int size);

enum vxge_hw_status __devinit vxge_hw_device_hw_info_get(
	void __iomem *bar0,
	struct vxge_hw_device_hw_info *hw_info);

enum vxge_hw_status
__vxge_hw_vpath_fw_ver_get(
	u32	vp_id,
	struct vxge_hw_vpath_reg __iomem *vpath_reg,
	struct vxge_hw_device_hw_info *hw_info);

enum vxge_hw_status
__vxge_hw_vpath_card_info_get(
	u32 vp_id,
	struct vxge_hw_vpath_reg __iomem *vpath_reg,
	struct vxge_hw_device_hw_info *hw_info);

enum vxge_hw_status __devinit vxge_hw_device_config_default_get(
	struct vxge_hw_device_config *device_config);

static inline
enum vxge_hw_device_link_state vxge_hw_device_link_state_get(
	struct __vxge_hw_device *devh)
{
	return devh->link_state;
}

void vxge_hw_device_terminate(struct __vxge_hw_device *devh);

const u8 *
vxge_hw_device_serial_number_get(struct __vxge_hw_device *devh);

u16 vxge_hw_device_link_width_get(struct __vxge_hw_device *devh);

const u8 *
vxge_hw_device_product_name_get(struct __vxge_hw_device *devh);

enum vxge_hw_status __devinit vxge_hw_device_initialize(
	struct __vxge_hw_device **devh,
	struct vxge_hw_device_attr *attr,
	struct vxge_hw_device_config *device_config);

enum vxge_hw_status vxge_hw_device_getpause_data(
	 struct __vxge_hw_device *devh,
	 u32 port,
	 u32 *tx,
	 u32 *rx);

enum vxge_hw_status vxge_hw_device_setpause_data(
	struct __vxge_hw_device *devh,
	u32 port,
	u32 tx,
	u32 rx);

static inline void *vxge_os_dma_malloc(struct pci_dev *pdev,
			unsigned long size,
			struct pci_dev **p_dmah,
			struct pci_dev **p_dma_acch)
{
	gfp_t flags;
	void *vaddr;
	unsigned long misaligned = 0;
	int realloc_flag = 0;
	*p_dma_acch = *p_dmah = NULL;

	if (in_interrupt())
		flags = GFP_ATOMIC | GFP_DMA;
	else
		flags = GFP_KERNEL | GFP_DMA;
realloc:
	vaddr = kmalloc((size), flags);
	if (vaddr == NULL)
		return vaddr;
	misaligned = (unsigned long)VXGE_ALIGN((unsigned long)vaddr,
				VXGE_CACHE_LINE_SIZE);
	if (realloc_flag)
		goto out;

	if (misaligned) {
		/* misaligned, free current one and try allocating
		 * size + VXGE_CACHE_LINE_SIZE memory
		 */
		kfree((void *) vaddr);
		size += VXGE_CACHE_LINE_SIZE;
		realloc_flag = 1;
		goto realloc;
	}
out:
	*(unsigned long *)p_dma_acch = misaligned;
	vaddr = (void *)((u8 *)vaddr + misaligned);
	return vaddr;
}

extern void vxge_hw_blockpool_block_add(
			struct __vxge_hw_device *devh,
			void *block_addr,
			u32 length,
			struct pci_dev *dma_h,
			struct pci_dev *acc_handle);

static inline void vxge_os_dma_malloc_async(struct pci_dev *pdev, void *devh,
					unsigned long size)
{
	gfp_t flags;
	void *vaddr;

	if (in_interrupt())
		flags = GFP_ATOMIC | GFP_DMA;
	else
		flags = GFP_KERNEL | GFP_DMA;

	vaddr = kmalloc((size), flags);

	vxge_hw_blockpool_block_add(devh, vaddr, size, pdev, pdev);
}

static inline void vxge_os_dma_free(struct pci_dev *pdev, const void *vaddr,
			struct pci_dev **p_dma_acch)
{
	unsigned long misaligned = *(unsigned long *)p_dma_acch;
	u8 *tmp = (u8 *)vaddr;
	tmp -= misaligned;
	kfree((void *)tmp);
}

static inline void*
__vxge_hw_mempool_item_priv(
	struct vxge_hw_mempool *mempool,
	u32 memblock_idx,
	void *item,
	u32 *memblock_item_idx)
{
	ptrdiff_t offset;
	void *memblock = mempool->memblocks_arr[memblock_idx];


	offset = (u32)((u8 *)item - (u8 *)memblock);
	vxge_assert(offset >= 0 && (u32)offset < mempool->memblock_size);

	(*memblock_item_idx) = (u32) offset / mempool->item_size;
	vxge_assert((*memblock_item_idx) < mempool->items_per_memblock);

	return (u8 *)mempool->memblocks_priv_arr[memblock_idx] +
			    (*memblock_item_idx) * mempool->items_priv_size;
}

enum vxge_hw_status
__vxge_hw_mempool_grow(
	struct vxge_hw_mempool *mempool,
	u32 num_allocate,
	u32 *num_allocated);

struct vxge_hw_mempool*
__vxge_hw_mempool_create(
	struct __vxge_hw_device *devh,
	u32 memblock_size,
	u32 item_size,
	u32 private_size,
	u32 items_initial,
	u32 items_max,
	struct vxge_hw_mempool_cbs *mp_callback,
	void *userdata);

struct __vxge_hw_channel*
__vxge_hw_channel_allocate(struct __vxge_hw_vpath_handle *vph,
			enum __vxge_hw_channel_type type, u32 length,
			u32 per_dtr_space, void *userdata);

void
__vxge_hw_channel_free(
	struct __vxge_hw_channel *channel);

enum vxge_hw_status
__vxge_hw_channel_initialize(
	struct __vxge_hw_channel *channel);

enum vxge_hw_status
__vxge_hw_channel_reset(
	struct __vxge_hw_channel *channel);

static inline struct __vxge_hw_fifo_txdl_priv *
__vxge_hw_fifo_txdl_priv(
	struct __vxge_hw_fifo *fifo,
	struct vxge_hw_fifo_txd *txdp)
{
	return (struct __vxge_hw_fifo_txdl_priv *)
			(((char *)((ulong)txdp->host_control)) +
				fifo->per_txdl_space);
}

enum vxge_hw_status vxge_hw_vpath_open(
	struct __vxge_hw_device *devh,
	struct vxge_hw_vpath_attr *attr,
	struct __vxge_hw_vpath_handle **vpath_handle);

enum vxge_hw_status
__vxge_hw_device_vpath_reset_in_prog_check(u64 __iomem *vpath_rst_in_prog);

enum vxge_hw_status vxge_hw_vpath_close(
	struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_status
vxge_hw_vpath_reset(
	struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_status
vxge_hw_vpath_recover_from_reset(
	struct __vxge_hw_vpath_handle *vpath_handle);

void
vxge_hw_vpath_enable(struct __vxge_hw_vpath_handle *vp);

enum vxge_hw_status
vxge_hw_vpath_check_leak(struct __vxge_hw_ring *ringh);

enum vxge_hw_status vxge_hw_vpath_mtu_set(
	struct __vxge_hw_vpath_handle *vpath_handle,
	u32 new_mtu);

enum vxge_hw_status vxge_hw_vpath_stats_enable(
	struct __vxge_hw_vpath_handle *vpath_handle);

enum vxge_hw_status
__vxge_hw_vpath_stats_access(
	struct __vxge_hw_virtualpath	*vpath,
	u32			operation,
	u32			offset,
	u64			*stat);

enum vxge_hw_status
__vxge_hw_vpath_xmac_tx_stats_get(
	struct __vxge_hw_virtualpath	*vpath,
	struct vxge_hw_xmac_vpath_tx_stats *vpath_tx_stats);

enum vxge_hw_status
__vxge_hw_vpath_xmac_rx_stats_get(
	struct __vxge_hw_virtualpath	*vpath,
	struct vxge_hw_xmac_vpath_rx_stats *vpath_rx_stats);

enum vxge_hw_status
__vxge_hw_vpath_stats_get(
	struct __vxge_hw_virtualpath *vpath,
	struct vxge_hw_vpath_stats_hw_info *hw_stats);

void
vxge_hw_vpath_rx_doorbell_init(struct __vxge_hw_vpath_handle *vp);

enum vxge_hw_status
__vxge_hw_device_vpath_config_check(struct vxge_hw_vp_config *vp_config);

void
__vxge_hw_device_pci_e_init(struct __vxge_hw_device *hldev);

enum vxge_hw_status
__vxge_hw_legacy_swapper_set(struct vxge_hw_legacy_reg __iomem *legacy_reg);

enum vxge_hw_status
__vxge_hw_vpath_swapper_set(struct vxge_hw_vpath_reg __iomem *vpath_reg);

enum vxge_hw_status
__vxge_hw_kdfc_swapper_set(struct vxge_hw_legacy_reg __iomem *legacy_reg,
	struct vxge_hw_vpath_reg __iomem *vpath_reg);

enum vxge_hw_status
__vxge_hw_device_register_poll(
	void __iomem	*reg,
	u64 mask, u32 max_millis);

#ifndef readq
static inline u64 readq(void __iomem *addr)
{
	u64 ret = 0;
	ret = readl(addr + 4);
	ret <<= 32;
	ret |= readl(addr);

	return ret;
}
#endif

#ifndef writeq
static inline void writeq(u64 val, void __iomem *addr)
{
	writel((u32) (val), addr);
	writel((u32) (val >> 32), (addr + 4));
}
#endif

static inline void __vxge_hw_pio_mem_write32_upper(u32 val, void __iomem *addr)
{
	writel(val, addr + 4);
}

static inline void __vxge_hw_pio_mem_write32_lower(u32 val, void __iomem *addr)
{
	writel(val, addr);
}

static inline enum vxge_hw_status
__vxge_hw_pio_mem_write64(u64 val64, void __iomem *addr,
			  u64 mask, u32 max_millis)
{
	enum vxge_hw_status status = VXGE_HW_OK;

	__vxge_hw_pio_mem_write32_lower((u32)vxge_bVALn(val64, 32, 32), addr);
	wmb();
	__vxge_hw_pio_mem_write32_upper((u32)vxge_bVALn(val64, 0, 32), addr);
	wmb();

	status = __vxge_hw_device_register_poll(addr, mask, max_millis);
	return status;
}

struct vxge_hw_toc_reg __iomem *
__vxge_hw_device_toc_get(void __iomem *bar0);

enum vxge_hw_status
__vxge_hw_device_reg_addr_get(struct __vxge_hw_device *hldev);

void
__vxge_hw_device_id_get(struct __vxge_hw_device *hldev);

void
__vxge_hw_device_host_info_get(struct __vxge_hw_device *hldev);

enum vxge_hw_status
vxge_hw_device_flick_link_led(struct __vxge_hw_device *devh, u64 on_off);

enum vxge_hw_status
__vxge_hw_device_initialize(struct __vxge_hw_device *hldev);

enum vxge_hw_status
__vxge_hw_vpath_pci_read(
	struct __vxge_hw_virtualpath	*vpath,
	u32			phy_func_0,
	u32			offset,
	u32			*val);

enum vxge_hw_status
__vxge_hw_vpath_addr_get(
	u32 vp_id,
	struct vxge_hw_vpath_reg __iomem *vpath_reg,
	u8 (macaddr)[ETH_ALEN],
	u8 (macaddr_mask)[ETH_ALEN]);

u32
__vxge_hw_vpath_func_id_get(
	u32 vp_id, struct vxge_hw_vpmgmt_reg __iomem *vpmgmt_reg);

enum vxge_hw_status
__vxge_hw_vpath_reset_check(struct __vxge_hw_virtualpath *vpath);

enum vxge_hw_status
vxge_hw_vpath_strip_fcs_check(struct __vxge_hw_device *hldev, u64 vpath_mask);

#define vxge_trace_aux(level, mask, fmt, ...) \
{\
		vxge_os_vaprintf(level, mask, fmt, __VA_ARGS__);\
}

#define vxge_debug(module, level, mask, fmt, ...) { \
if ((level >= VXGE_TRACE && ((module & VXGE_DEBUG_TRACE_MASK) == module)) || \
	(level >= VXGE_ERR && ((module & VXGE_DEBUG_ERR_MASK) == module))) {\
	if ((mask & VXGE_DEBUG_MASK) == mask)\
		vxge_trace_aux(level, mask, fmt, __VA_ARGS__); \
} \
}

#if (VXGE_COMPONENT_LL & VXGE_DEBUG_MODULE_MASK)
#define vxge_debug_ll(level, mask, fmt, ...) \
{\
	vxge_debug(VXGE_COMPONENT_LL, level, mask, fmt, __VA_ARGS__);\
}

#else
#define vxge_debug_ll(level, mask, fmt, ...)
#endif

enum vxge_hw_status vxge_hw_vpath_rts_rth_itable_set(
			struct __vxge_hw_vpath_handle **vpath_handles,
			u32 vpath_count,
			u8 *mtable,
			u8 *itable,
			u32 itable_size);

enum vxge_hw_status vxge_hw_vpath_rts_rth_set(
	struct __vxge_hw_vpath_handle *vpath_handle,
	enum vxge_hw_rth_algoritms algorithm,
	struct vxge_hw_rth_hash_types *hash_type,
	u16 bucket_size);

enum vxge_hw_status
__vxge_hw_device_is_privilaged(u32 host_type, u32 func_id);
#endif
