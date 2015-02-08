

/* Common definitions for all Efx net driver code */

#ifndef EFX_NET_DRIVER_H
#define EFX_NET_DRIVER_H

#include <linux/version.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>
#include <linux/mdio.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/device.h>
#include <linux/highmem.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>

#include "enum.h"
#include "bitfield.h"

#ifndef EFX_DRIVER_NAME
#define EFX_DRIVER_NAME	"sfc"
#endif
#define EFX_DRIVER_VERSION	"3.0"

#ifdef EFX_ENABLE_DEBUG
#define EFX_BUG_ON_PARANOID(x) BUG_ON(x)
#define EFX_WARN_ON_PARANOID(x) WARN_ON(x)
#else
#define EFX_BUG_ON_PARANOID(x) do {} while (0)
#define EFX_WARN_ON_PARANOID(x) do {} while (0)
#endif

/* Un-rate-limited logging */
#define EFX_ERR(efx, fmt, args...) \
dev_err(&((efx)->pci_dev->dev), "ERR: %s " fmt, efx_dev_name(efx), ##args)

#define EFX_INFO(efx, fmt, args...) \
dev_info(&((efx)->pci_dev->dev), "INFO: %s " fmt, efx_dev_name(efx), ##args)

#ifdef EFX_ENABLE_DEBUG
#define EFX_LOG(efx, fmt, args...) \
dev_info(&((efx)->pci_dev->dev), "DBG: %s " fmt, efx_dev_name(efx), ##args)
#else
#define EFX_LOG(efx, fmt, args...) \
dev_dbg(&((efx)->pci_dev->dev), "DBG: %s " fmt, efx_dev_name(efx), ##args)
#endif

#define EFX_TRACE(efx, fmt, args...) do {} while (0)

#define EFX_REGDUMP(efx, fmt, args...) do {} while (0)

/* Rate-limited logging */
#define EFX_ERR_RL(efx, fmt, args...) \
do {if (net_ratelimit()) EFX_ERR(efx, fmt, ##args); } while (0)

#define EFX_INFO_RL(efx, fmt, args...) \
do {if (net_ratelimit()) EFX_INFO(efx, fmt, ##args); } while (0)

#define EFX_LOG_RL(efx, fmt, args...) \
do {if (net_ratelimit()) EFX_LOG(efx, fmt, ##args); } while (0)


#define EFX_MAX_CHANNELS 32
#define EFX_MAX_RX_QUEUES EFX_MAX_CHANNELS

#define EFX_MAX_CORE_TX_QUEUES	EFX_MAX_CHANNELS
#define EFX_TXQ_TYPE_OFFLOAD	1
#define EFX_TXQ_TYPES		2
#define EFX_MAX_TX_QUEUES	(EFX_TXQ_TYPES * EFX_MAX_CORE_TX_QUEUES)

struct efx_special_buffer {
	void *addr;
	dma_addr_t dma_addr;
	unsigned int len;
	int index;
	int entries;
};

enum efx_flush_state {
	FLUSH_NONE,
	FLUSH_PENDING,
	FLUSH_FAILED,
	FLUSH_DONE,
};

struct efx_tx_buffer {
	const struct sk_buff *skb;
	struct efx_tso_header *tsoh;
	dma_addr_t dma_addr;
	unsigned short len;
	bool continuation;
	bool unmap_single;
	unsigned short unmap_len;
};

struct efx_tx_queue {
	/* Members which don't change on the fast path */
	struct efx_nic *efx ____cacheline_aligned_in_smp;
	unsigned queue;
	struct efx_channel *channel;
	struct efx_nic *nic;
	struct efx_tx_buffer *buffer;
	struct efx_special_buffer txd;
	enum efx_flush_state flushed;

	/* Members used mainly on the completion path */
	unsigned int read_count ____cacheline_aligned_in_smp;
	int stopped;

	/* Members used only on the xmit path */
	unsigned int insert_count ____cacheline_aligned_in_smp;
	unsigned int write_count;
	unsigned int old_read_count;
	struct efx_tso_header *tso_headers_free;
	unsigned int tso_bursts;
	unsigned int tso_long_headers;
	unsigned int tso_packets;
};

struct efx_rx_buffer {
	dma_addr_t dma_addr;
	struct sk_buff *skb;
	struct page *page;
	char *data;
	unsigned int len;
	dma_addr_t unmap_addr;
};

struct efx_rx_queue {
	struct efx_nic *efx;
	int queue;
	struct efx_channel *channel;
	struct efx_rx_buffer *buffer;
	struct efx_special_buffer rxd;

	int added_count;
	int notified_count;
	int removed_count;
	spinlock_t add_lock;
	unsigned int max_fill;
	unsigned int fast_fill_trigger;
	unsigned int fast_fill_limit;
	unsigned int min_fill;
	unsigned int min_overfill;
	unsigned int alloc_page_count;
	unsigned int alloc_skb_count;
	struct delayed_work work;
	unsigned int slow_fill_count;

	struct page *buf_page;
	dma_addr_t buf_dma_addr;
	char *buf_data;
	enum efx_flush_state flushed;
};

struct efx_buffer {
	void *addr;
	dma_addr_t dma_addr;
	unsigned int len;
};


enum efx_rx_alloc_method {
	RX_ALLOC_METHOD_AUTO = 0,
	RX_ALLOC_METHOD_SKB = 1,
	RX_ALLOC_METHOD_PAGE = 2,
};

struct efx_channel {
	struct efx_nic *efx;
	int channel;
	char name[IFNAMSIZ + 6];
	bool enabled;
	int irq;
	unsigned int irq_moderation;
	struct net_device *napi_dev;
	struct napi_struct napi_str;
	bool work_pending;
	struct efx_special_buffer eventq;
	unsigned int eventq_read_ptr;
	unsigned int last_eventq_read_ptr;
	unsigned int eventq_magic;

	unsigned int irq_count;
	unsigned int irq_mod_score;

	int rx_alloc_level;
	int rx_alloc_push_pages;

	unsigned n_rx_tobe_disc;
	unsigned n_rx_ip_hdr_chksum_err;
	unsigned n_rx_tcp_udp_chksum_err;
	unsigned n_rx_mcast_mismatch;
	unsigned n_rx_frm_trunc;
	unsigned n_rx_overlength;
	unsigned n_skbuff_leaks;

	/* Used to pipeline received packets in order to optimise memory
	 * access with prefetches.
	 */
	struct efx_rx_buffer *rx_pkt;
	bool rx_pkt_csummed;

	struct efx_tx_queue *tx_queue;
	atomic_t tx_stop_count;
	spinlock_t tx_stop_lock;
};

enum efx_led_mode {
	EFX_LED_OFF	= 0,
	EFX_LED_ON	= 1,
	EFX_LED_DEFAULT	= 2
};

#define STRING_TABLE_LOOKUP(val, member) \
	((val) < member ## _max) ? member ## _names[val] : "(invalid)"

extern const char *efx_loopback_mode_names[];
extern const unsigned int efx_loopback_mode_max;
#define LOOPBACK_MODE(efx) \
	STRING_TABLE_LOOKUP((efx)->loopback_mode, efx_loopback_mode)

extern const char *efx_interrupt_mode_names[];
extern const unsigned int efx_interrupt_mode_max;
#define INT_MODE(efx) \
	STRING_TABLE_LOOKUP(efx->interrupt_mode, efx_interrupt_mode)

extern const char *efx_reset_type_names[];
extern const unsigned int efx_reset_type_max;
#define RESET_TYPE(type) \
	STRING_TABLE_LOOKUP(type, efx_reset_type)

enum efx_int_mode {
	/* Be careful if altering to correct macro below */
	EFX_INT_MODE_MSIX = 0,
	EFX_INT_MODE_MSI = 1,
	EFX_INT_MODE_LEGACY = 2,
	EFX_INT_MODE_MAX	/* Insert any new items before this */
};
#define EFX_INT_MODE_USE_MSI(x) (((x)->interrupt_mode) <= EFX_INT_MODE_MSI)

#define EFX_IS10G(efx) ((efx)->link_state.speed == 10000)

enum nic_state {
	STATE_INIT = 0,
	STATE_RUNNING = 1,
	STATE_FINI = 2,
	STATE_DISABLED = 3,
	STATE_MAX,
};

#ifdef CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS
#define EFX_PAGE_IP_ALIGN 0
#else
#define EFX_PAGE_IP_ALIGN NET_IP_ALIGN
#endif

#define EFX_PAGE_SKB_ALIGN 2

/* Forward declaration */
struct efx_nic;

/* Pseudo bit-mask flow control field */
enum efx_fc_type {
	EFX_FC_RX = FLOW_CTRL_RX,
	EFX_FC_TX = FLOW_CTRL_TX,
	EFX_FC_AUTO = 4,
};

struct efx_link_state {
	bool up;
	bool fd;
	enum efx_fc_type fc;
	unsigned int speed;
};

static inline bool efx_link_state_equal(const struct efx_link_state *left,
					const struct efx_link_state *right)
{
	return left->up == right->up && left->fd == right->fd &&
		left->fc == right->fc && left->speed == right->speed;
}

struct efx_mac_operations {
	int (*reconfigure) (struct efx_nic *efx);
	void (*update_stats) (struct efx_nic *efx);
	bool (*check_fault)(struct efx_nic *efx);
};

struct efx_phy_operations {
	int (*probe) (struct efx_nic *efx);
	int (*init) (struct efx_nic *efx);
	void (*fini) (struct efx_nic *efx);
	void (*remove) (struct efx_nic *efx);
	int (*reconfigure) (struct efx_nic *efx);
	bool (*poll) (struct efx_nic *efx);
	void (*get_settings) (struct efx_nic *efx,
			      struct ethtool_cmd *ecmd);
	int (*set_settings) (struct efx_nic *efx,
			     struct ethtool_cmd *ecmd);
	void (*set_npage_adv) (struct efx_nic *efx, u32);
	int (*test_alive) (struct efx_nic *efx);
	const char *(*test_name) (struct efx_nic *efx, unsigned int index);
	int (*run_tests) (struct efx_nic *efx, int *results, unsigned flags);
};

enum efx_phy_mode {
	PHY_MODE_NORMAL		= 0,
	PHY_MODE_TX_DISABLED	= 1,
	PHY_MODE_LOW_POWER	= 2,
	PHY_MODE_OFF		= 4,
	PHY_MODE_SPECIAL	= 8,
};

static inline bool efx_phy_mode_disabled(enum efx_phy_mode mode)
{
	return !!(mode & ~PHY_MODE_TX_DISABLED);
}

struct efx_mac_stats {
	u64 tx_bytes;
	u64 tx_good_bytes;
	u64 tx_bad_bytes;
	unsigned long tx_packets;
	unsigned long tx_bad;
	unsigned long tx_pause;
	unsigned long tx_control;
	unsigned long tx_unicast;
	unsigned long tx_multicast;
	unsigned long tx_broadcast;
	unsigned long tx_lt64;
	unsigned long tx_64;
	unsigned long tx_65_to_127;
	unsigned long tx_128_to_255;
	unsigned long tx_256_to_511;
	unsigned long tx_512_to_1023;
	unsigned long tx_1024_to_15xx;
	unsigned long tx_15xx_to_jumbo;
	unsigned long tx_gtjumbo;
	unsigned long tx_collision;
	unsigned long tx_single_collision;
	unsigned long tx_multiple_collision;
	unsigned long tx_excessive_collision;
	unsigned long tx_deferred;
	unsigned long tx_late_collision;
	unsigned long tx_excessive_deferred;
	unsigned long tx_non_tcpudp;
	unsigned long tx_mac_src_error;
	unsigned long tx_ip_src_error;
	u64 rx_bytes;
	u64 rx_good_bytes;
	u64 rx_bad_bytes;
	unsigned long rx_packets;
	unsigned long rx_good;
	unsigned long rx_bad;
	unsigned long rx_pause;
	unsigned long rx_control;
	unsigned long rx_unicast;
	unsigned long rx_multicast;
	unsigned long rx_broadcast;
	unsigned long rx_lt64;
	unsigned long rx_64;
	unsigned long rx_65_to_127;
	unsigned long rx_128_to_255;
	unsigned long rx_256_to_511;
	unsigned long rx_512_to_1023;
	unsigned long rx_1024_to_15xx;
	unsigned long rx_15xx_to_jumbo;
	unsigned long rx_gtjumbo;
	unsigned long rx_bad_lt64;
	unsigned long rx_bad_64_to_15xx;
	unsigned long rx_bad_15xx_to_jumbo;
	unsigned long rx_bad_gtjumbo;
	unsigned long rx_overflow;
	unsigned long rx_missed;
	unsigned long rx_false_carrier;
	unsigned long rx_symbol_error;
	unsigned long rx_align_error;
	unsigned long rx_length_error;
	unsigned long rx_internal_error;
	unsigned long rx_good_lt64;
};

/* Number of bits used in a multicast filter hash address */
#define EFX_MCAST_HASH_BITS 8

/* Number of (single-bit) entries in a multicast filter hash */
#define EFX_MCAST_HASH_ENTRIES (1 << EFX_MCAST_HASH_BITS)

/* An Efx multicast filter hash */
union efx_multicast_hash {
	u8 byte[EFX_MCAST_HASH_ENTRIES / 8];
	efx_oword_t oword[EFX_MCAST_HASH_ENTRIES / sizeof(efx_oword_t) / 8];
};

struct efx_nic {
	char name[IFNAMSIZ];
	struct pci_dev *pci_dev;
	const struct efx_nic_type *type;
	int legacy_irq;
	struct workqueue_struct *workqueue;
	char workqueue_name[16];
	struct work_struct reset_work;
	struct delayed_work monitor_work;
	resource_size_t membase_phys;
	void __iomem *membase;
	spinlock_t biu_lock;
	enum efx_int_mode interrupt_mode;
	bool irq_rx_adaptive;
	unsigned int irq_rx_moderation;

	enum nic_state state;
	enum reset_type reset_pending;

	struct efx_tx_queue tx_queue[EFX_MAX_TX_QUEUES];
	struct efx_rx_queue rx_queue[EFX_MAX_RX_QUEUES];
	struct efx_channel channel[EFX_MAX_CHANNELS];

	unsigned next_buffer_table;
	unsigned n_channels;
	unsigned n_rx_channels;
	unsigned n_tx_channels;
	unsigned int rx_buffer_len;
	unsigned int rx_buffer_order;

	unsigned int_error_count;
	unsigned long int_error_expire;

	struct efx_buffer irq_status;
	volatile signed int last_irq_cpu;
	unsigned irq_zero_count;
	unsigned fatal_irq_level;

	struct efx_spi_device *spi_flash;
	struct efx_spi_device *spi_eeprom;
	struct mutex spi_lock;
#ifdef CONFIG_SFC_MTD
	struct list_head mtd_list;
#endif

	unsigned n_rx_nodesc_drop_cnt;

	void *nic_data;

	struct mutex mac_lock;
	struct work_struct mac_work;
	bool port_enabled;
	bool port_inhibited;

	bool port_initialized;
	struct net_device *net_dev;
	bool rx_checksum_enabled;

	struct efx_mac_stats mac_stats;
	struct efx_buffer stats_buffer;
	spinlock_t stats_lock;

	struct efx_mac_operations *mac_op;
	unsigned char mac_address[ETH_ALEN];

	unsigned int phy_type;
	struct mutex mdio_lock;
	struct efx_phy_operations *phy_op;
	void *phy_data;
	struct mdio_if_info mdio;
	unsigned int mdio_bus;
	enum efx_phy_mode phy_mode;

	bool xmac_poll_required;
	u32 link_advertising;
	struct efx_link_state link_state;
	unsigned int n_link_state_changes;

	bool promiscuous;
	union efx_multicast_hash multicast_hash;
	enum efx_fc_type wanted_fc;

	atomic_t rx_reset;
	enum efx_loopback_mode loopback_mode;
	u64 loopback_modes;

	void *loopback_selftest;
};

static inline int efx_dev_registered(struct efx_nic *efx)
{
	return efx->net_dev->reg_state == NETREG_REGISTERED;
}

static inline const char *efx_dev_name(struct efx_nic *efx)
{
	return efx_dev_registered(efx) ? efx->name : "";
}

static inline unsigned int efx_port_num(struct efx_nic *efx)
{
	return efx->net_dev->dev_id;
}

struct efx_nic_type {
	int (*probe)(struct efx_nic *efx);
	void (*remove)(struct efx_nic *efx);
	int (*init)(struct efx_nic *efx);
	void (*fini)(struct efx_nic *efx);
	void (*monitor)(struct efx_nic *efx);
	int (*reset)(struct efx_nic *efx, enum reset_type method);
	int (*probe_port)(struct efx_nic *efx);
	void (*remove_port)(struct efx_nic *efx);
	void (*prepare_flush)(struct efx_nic *efx);
	void (*update_stats)(struct efx_nic *efx);
	void (*start_stats)(struct efx_nic *efx);
	void (*stop_stats)(struct efx_nic *efx);
	void (*set_id_led)(struct efx_nic *efx, enum efx_led_mode mode);
	void (*push_irq_moderation)(struct efx_channel *channel);
	void (*push_multicast_hash)(struct efx_nic *efx);
	int (*reconfigure_port)(struct efx_nic *efx);
	void (*get_wol)(struct efx_nic *efx, struct ethtool_wolinfo *wol);
	int (*set_wol)(struct efx_nic *efx, u32 type);
	void (*resume_wol)(struct efx_nic *efx);
	int (*test_registers)(struct efx_nic *efx);
	int (*test_nvram)(struct efx_nic *efx);
	struct efx_mac_operations *default_mac_ops;

	int revision;
	unsigned int mem_map_size;
	unsigned int txd_ptr_tbl_base;
	unsigned int rxd_ptr_tbl_base;
	unsigned int buf_tbl_base;
	unsigned int evq_ptr_tbl_base;
	unsigned int evq_rptr_tbl_base;
	u64 max_dma_mask;
	unsigned int rx_buffer_padding;
	unsigned int max_interrupt_mode;
	unsigned int phys_addr_channels;
	unsigned int tx_dc_base;
	unsigned int rx_dc_base;
	unsigned long offload_features;
	u32 reset_world_flags;
};


/* Iterate over all used channels */
#define efx_for_each_channel(_channel, _efx)				\
	for (_channel = &((_efx)->channel[0]);				\
	     _channel < &((_efx)->channel[(efx)->n_channels]);		\
	     _channel++)

/* Iterate over all used TX queues */
#define efx_for_each_tx_queue(_tx_queue, _efx)				\
	for (_tx_queue = &((_efx)->tx_queue[0]);			\
	     _tx_queue < &((_efx)->tx_queue[EFX_TXQ_TYPES *		\
					    (_efx)->n_tx_channels]);	\
	     _tx_queue++)

/* Iterate over all TX queues belonging to a channel */
#define efx_for_each_channel_tx_queue(_tx_queue, _channel)		\
	for (_tx_queue = (_channel)->tx_queue;				\
	     _tx_queue && _tx_queue < (_channel)->tx_queue + EFX_TXQ_TYPES; \
	     _tx_queue++)

/* Iterate over all used RX queues */
#define efx_for_each_rx_queue(_rx_queue, _efx)				\
	for (_rx_queue = &((_efx)->rx_queue[0]);			\
	     _rx_queue < &((_efx)->rx_queue[(_efx)->n_rx_channels]);	\
	     _rx_queue++)

/* Iterate over all RX queues belonging to a channel */
#define efx_for_each_channel_rx_queue(_rx_queue, _channel)		\
	for (_rx_queue = &((_channel)->efx->rx_queue[(_channel)->channel]); \
	     _rx_queue;							\
	     _rx_queue = NULL)						\
		if (_rx_queue->channel != (_channel))			\
			continue;					\
		else

static inline struct efx_rx_buffer *efx_rx_buffer(struct efx_rx_queue *rx_queue,
						  unsigned int index)
{
	return (&rx_queue->buffer[index]);
}

/* Set bit in a little-endian bitfield */
static inline void set_bit_le(unsigned nr, unsigned char *addr)
{
	addr[nr / 8] |= (1 << (nr % 8));
}

/* Clear bit in a little-endian bitfield */
static inline void clear_bit_le(unsigned nr, unsigned char *addr)
{
	addr[nr / 8] &= ~(1 << (nr % 8));
}


#define EFX_MAX_FRAME_LEN(mtu) \
	((((mtu) + ETH_HLEN + VLAN_HLEN + 4/* FCS */ + 7) & ~7) + 16)


#endif /* EFX_NET_DRIVER_H */
