


#ifndef RT2X00QUEUE_H
#define RT2X00QUEUE_H

#include <linux/prefetch.h>

#define DATA_FRAME_SIZE		2432
#define MGMT_FRAME_SIZE		256
#define AGGREGATION_SIZE	3840

#define RX_ENTRIES	24
#define TX_ENTRIES	24
#define BEACON_ENTRIES	1
#define ATIM_ENTRIES	8

enum data_queue_qid {
	QID_AC_BE = 0,
	QID_AC_BK = 1,
	QID_AC_VI = 2,
	QID_AC_VO = 3,
	QID_HCCA = 4,
	QID_MGMT = 13,
	QID_RX = 14,
	QID_OTHER = 15,
	QID_BEACON,
	QID_ATIM,
};

enum skb_frame_desc_flags {
	SKBDESC_DMA_MAPPED_RX = 1 << 0,
	SKBDESC_DMA_MAPPED_TX = 1 << 1,
	SKBDESC_IV_STRIPPED = 1 << 2,
	SKBDESC_NOT_MAC80211 = 1 << 3,
	SKBDESC_DESC_IN_SKB = 1 << 4,
};

struct skb_frame_desc {
	u8 flags;

	u8 desc_len;
	u8 tx_rate_idx;
	u8 tx_rate_flags;

	void *desc;

	__le32 iv[2];

	dma_addr_t skb_dma;

	struct queue_entry *entry;
};

static inline struct skb_frame_desc* get_skb_frame_desc(struct sk_buff *skb)
{
	BUILD_BUG_ON(sizeof(struct skb_frame_desc) >
		     IEEE80211_TX_INFO_DRIVER_DATA_SIZE);
	return (struct skb_frame_desc *)&IEEE80211_SKB_CB(skb)->driver_data;
}

enum rxdone_entry_desc_flags {
	RXDONE_SIGNAL_PLCP = BIT(0),
	RXDONE_SIGNAL_BITRATE = BIT(1),
	RXDONE_SIGNAL_MCS = BIT(2),
	RXDONE_MY_BSS = BIT(3),
	RXDONE_CRYPTO_IV = BIT(4),
	RXDONE_CRYPTO_ICV = BIT(5),
	RXDONE_L2PAD = BIT(6),
};

#define RXDONE_SIGNAL_MASK \
	( RXDONE_SIGNAL_PLCP | RXDONE_SIGNAL_BITRATE | RXDONE_SIGNAL_MCS )

struct rxdone_entry_desc {
	u64 timestamp;
	int signal;
	int rssi;
	int size;
	int flags;
	int dev_flags;
	u16 rate_mode;
	u8 cipher;
	u8 cipher_status;

	__le32 iv[2];
	__le32 icv;
};

enum txdone_entry_desc_flags {
	TXDONE_UNKNOWN,
	TXDONE_SUCCESS,
	TXDONE_FALLBACK,
	TXDONE_FAILURE,
	TXDONE_EXCESSIVE_RETRY,
};

struct txdone_entry_desc {
	unsigned long flags;
	int retry;
};

enum txentry_desc_flags {
	ENTRY_TXD_RTS_FRAME,
	ENTRY_TXD_CTS_FRAME,
	ENTRY_TXD_GENERATE_SEQ,
	ENTRY_TXD_FIRST_FRAGMENT,
	ENTRY_TXD_MORE_FRAG,
	ENTRY_TXD_REQ_TIMESTAMP,
	ENTRY_TXD_BURST,
	ENTRY_TXD_ACK,
	ENTRY_TXD_RETRY_MODE,
	ENTRY_TXD_ENCRYPT,
	ENTRY_TXD_ENCRYPT_PAIRWISE,
	ENTRY_TXD_ENCRYPT_IV,
	ENTRY_TXD_ENCRYPT_MMIC,
	ENTRY_TXD_HT_AMPDU,
	ENTRY_TXD_HT_BW_40,
	ENTRY_TXD_HT_SHORT_GI,
};

struct txentry_desc {
	unsigned long flags;

	enum data_queue_qid queue;

	u16 length;
	u16 header_length;

	u16 length_high;
	u16 length_low;
	u16 signal;
	u16 service;

	u16 mcs;
	u16 stbc;
	u16 ba_size;
	u16 rate_mode;
	u16 mpdu_density;

	short retry_limit;
	short aifs;
	short ifs;
	short txop;
	short cw_min;
	short cw_max;

	enum cipher cipher;
	u16 key_idx;
	u16 iv_offset;
	u16 iv_len;
};

enum queue_entry_flags {
	ENTRY_BCN_ASSIGNED,
	ENTRY_OWNER_DEVICE_DATA,
	ENTRY_OWNER_DEVICE_CRYPTO,
	ENTRY_DATA_PENDING,
};

struct queue_entry {
	unsigned long flags;

	struct data_queue *queue;

	struct sk_buff *skb;

	unsigned int entry_idx;

	void *priv_data;
};

enum queue_index {
	Q_INDEX,
	Q_INDEX_DONE,
	Q_INDEX_CRYPTO,
	Q_INDEX_MAX,
};

struct data_queue {
	struct rt2x00_dev *rt2x00dev;
	struct queue_entry *entries;

	enum data_queue_qid qid;

	spinlock_t lock;
	unsigned int count;
	unsigned short limit;
	unsigned short threshold;
	unsigned short length;
	unsigned short index[Q_INDEX_MAX];

	unsigned short txop;
	unsigned short aifs;
	unsigned short cw_min;
	unsigned short cw_max;

	unsigned short data_size;
	unsigned short desc_size;

	unsigned short usb_endpoint;
	unsigned short usb_maxpacket;
};

struct data_queue_desc {
	unsigned short entry_num;
	unsigned short data_size;
	unsigned short desc_size;
	unsigned short priv_size;
};

#define queue_end(__dev) \
	&(__dev)->rx[(__dev)->data_queues]

#define tx_queue_end(__dev) \
	&(__dev)->tx[(__dev)->ops->tx_queues]

#define queue_next(__queue) \
	&(__queue)[1]

#define queue_loop(__entry, __start, __end)			\
	for ((__entry) = (__start);				\
	     prefetch(queue_next(__entry)), (__entry) != (__end);\
	     (__entry) = queue_next(__entry))

#define queue_for_each(__dev, __entry) \
	queue_loop(__entry, (__dev)->rx, queue_end(__dev))

#define tx_queue_for_each(__dev, __entry) \
	queue_loop(__entry, (__dev)->tx, tx_queue_end(__dev))

#define txall_queue_for_each(__dev, __entry) \
	queue_loop(__entry, (__dev)->tx, queue_end(__dev))

static inline int rt2x00queue_empty(struct data_queue *queue)
{
	return queue->length == 0;
}

static inline int rt2x00queue_full(struct data_queue *queue)
{
	return queue->length == queue->limit;
}

static inline int rt2x00queue_available(struct data_queue *queue)
{
	return queue->limit - queue->length;
}

static inline int rt2x00queue_threshold(struct data_queue *queue)
{
	return rt2x00queue_available(queue) < queue->threshold;
}

static inline void _rt2x00_desc_read(__le32 *desc, const u8 word, __le32 *value)
{
	*value = desc[word];
}

static inline void rt2x00_desc_read(__le32 *desc, const u8 word, u32 *value)
{
	__le32 tmp;
	_rt2x00_desc_read(desc, word, &tmp);
	*value = le32_to_cpu(tmp);
}

static inline void _rt2x00_desc_write(__le32 *desc, const u8 word, __le32 value)
{
	desc[word] = value;
}

static inline void rt2x00_desc_write(__le32 *desc, const u8 word, u32 value)
{
	_rt2x00_desc_write(desc, word, cpu_to_le32(value));
}

#endif /* RT2X00QUEUE_H */
