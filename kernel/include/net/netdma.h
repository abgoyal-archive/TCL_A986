
#ifndef NETDMA_H
#define NETDMA_H
#ifdef CONFIG_NET_DMA
#include <linux/dmaengine.h>
#include <linux/skbuff.h>

int dma_skb_copy_datagram_iovec(struct dma_chan* chan,
		struct sk_buff *skb, int offset, struct iovec *to,
		size_t len, struct dma_pinned_list *pinned_list);

#endif /* CONFIG_NET_DMA */
#endif /* NETDMA_H */
