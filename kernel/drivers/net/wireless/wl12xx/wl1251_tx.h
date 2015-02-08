

#ifndef __WL1251_TX_H__
#define __WL1251_TX_H__

#include <linux/bitops.h>
#include "wl1251_acx.h"


#define TX_COMPLETE_REQUIRED_BIT	0x80
#define TX_STATUS_DATA_OUT_COUNT_MASK   0xf

#define WL1251_TX_ALIGN_TO 4
#define WL1251_TX_ALIGN(len) (((len) + WL1251_TX_ALIGN_TO - 1) & \
			     ~(WL1251_TX_ALIGN_TO - 1))
#define WL1251_TKIP_IV_SPACE 4

struct tx_control {
	/* Rate Policy (class) index */
	unsigned rate_policy:3;

	/* When set, no ack policy is expected */
	unsigned ack_policy:1;

	/*
	 * Packet type:
	 * 0 -> 802.11
	 * 1 -> 802.3
	 * 2 -> IP
	 * 3 -> raw codec
	 */
	unsigned packet_type:2;

	/* If set, this is a QoS-Null or QoS-Data frame */
	unsigned qos:1;

	/*
	 * If set, the target triggers the tx complete INT
	 * upon frame sending completion.
	 */
	unsigned tx_complete:1;

	/* 2 bytes padding before packet header */
	unsigned xfer_pad:1;

	unsigned reserved:7;
} __attribute__ ((packed));


struct tx_double_buffer_desc {
	/* Length of payload, including headers. */
	u16 length;

	/*
	 * A bit mask that specifies the initial rate to be used
	 * Possible values are:
	 * 0x0001 - 1Mbits
	 * 0x0002 - 2Mbits
	 * 0x0004 - 5.5Mbits
	 * 0x0008 - 6Mbits
	 * 0x0010 - 9Mbits
	 * 0x0020 - 11Mbits
	 * 0x0040 - 12Mbits
	 * 0x0080 - 18Mbits
	 * 0x0100 - 22Mbits
	 * 0x0200 - 24Mbits
	 * 0x0400 - 36Mbits
	 * 0x0800 - 48Mbits
	 * 0x1000 - 54Mbits
	 */
	u16 rate;

	/* Time in us that a packet can spend in the target */
	u32 expiry_time;

	/* index of the TX queue used for this packet */
	u8 xmit_queue;

	/* Used to identify a packet */
	u8 id;

	struct tx_control control;

	/*
	 * The FW should cut the packet into fragments
	 * of this size.
	 */
	u16 frag_threshold;

	/* Numbers of HW queue blocks to be allocated */
	u8 num_mem_blocks;

	u8 reserved;
} __attribute__ ((packed));

enum {
	TX_SUCCESS              = 0,
	TX_DMA_ERROR            = BIT(7),
	TX_DISABLED             = BIT(6),
	TX_RETRY_EXCEEDED       = BIT(5),
	TX_TIMEOUT              = BIT(4),
	TX_KEY_NOT_FOUND        = BIT(3),
	TX_ENCRYPT_FAIL         = BIT(2),
	TX_UNAVAILABLE_PRIORITY = BIT(1),
};

struct tx_result {
	/*
	 * Ownership synchronization between the host and
	 * the firmware. If done_1 and done_2 are cleared,
	 * owned by the FW (no info ready).
	 */
	u8 done_1;

	/* same as double_buffer_desc->id */
	u8 id;

	/*
	 * Total air access duration consumed by this
	 * packet, including all retries and overheads.
	 */
	u16 medium_usage;

	/* Total media delay (from 1st EDCA AIFS counter until TX Complete). */
	u32 medium_delay;

	/* Time between host xfer and tx complete */
	u32 fw_hnadling_time;

	/* The LS-byte of the last TKIP sequence number. */
	u8 lsb_seq_num;

	/* Retry count */
	u8 ack_failures;

	/* At which rate we got a ACK */
	u16 rate;

	u16 reserved;

	/* TX_* */
	u8 status;

	/* See done_1 */
	u8 done_2;
} __attribute__ ((packed));

static inline int wl1251_tx_get_queue(int queue)
{
	switch (queue) {
	case 0:
		return QOS_AC_VO;
	case 1:
		return QOS_AC_VI;
	case 2:
		return QOS_AC_BE;
	case 3:
		return QOS_AC_BK;
	default:
		return QOS_AC_BE;
	}
}

void wl1251_tx_work(struct work_struct *work);
void wl1251_tx_complete(struct wl1251 *wl);
void wl1251_tx_flush(struct wl1251 *wl);

#endif
