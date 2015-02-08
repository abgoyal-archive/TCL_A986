

#ifndef __IWM_RX_H__
#define __IWM_RX_H__

#include <linux/skbuff.h>

#include "umac.h"

struct iwm_rx_ticket_node {
	struct list_head node;
	struct iwm_rx_ticket *ticket;
};

struct iwm_rx_packet {
	struct list_head node;
	u16 id;
	struct sk_buff *skb;
	unsigned long pkt_size;
};

void iwm_rx_worker(struct work_struct *work);

#endif
