

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
#include "omap2430.h"
#endif

#ifndef CONFIG_BLACKFIN

#define MUSB_HSDMA_BASE		0x200
#define MUSB_HSDMA_INTR		(MUSB_HSDMA_BASE + 0)
#define MUSB_HSDMA_CONTROL		0x4
#define MUSB_HSDMA_ADDRESS		0x8
#define MUSB_HSDMA_COUNT		0xc

#define MUSB_HSDMA_CHANNEL_OFFSET(_bchannel, _offset)		\
		(MUSB_HSDMA_BASE + (_bchannel << 4) + _offset)

#define musb_read_hsdma_addr(mbase, bchannel)	\
	musb_readl(mbase,	\
		   MUSB_HSDMA_CHANNEL_OFFSET(bchannel, MUSB_HSDMA_ADDRESS))

#define musb_write_hsdma_addr(mbase, bchannel, addr) \
	musb_writel(mbase, \
		    MUSB_HSDMA_CHANNEL_OFFSET(bchannel, MUSB_HSDMA_ADDRESS), \
		    addr)

#define musb_read_hsdma_count(mbase, bchannel)	\
	musb_readl(mbase,	\
		   MUSB_HSDMA_CHANNEL_OFFSET(bchannel, MUSB_HSDMA_COUNT))

#define musb_write_hsdma_count(mbase, bchannel, len) \
	musb_writel(mbase, \
		    MUSB_HSDMA_CHANNEL_OFFSET(bchannel, MUSB_HSDMA_COUNT), \
		    len)
#else

#define MUSB_HSDMA_BASE		0x400
#define MUSB_HSDMA_INTR		(MUSB_HSDMA_BASE + 0)
#define MUSB_HSDMA_CONTROL		0x04
#define MUSB_HSDMA_ADDR_LOW		0x08
#define MUSB_HSDMA_ADDR_HIGH		0x0C
#define MUSB_HSDMA_COUNT_LOW		0x10
#define MUSB_HSDMA_COUNT_HIGH		0x14

#define MUSB_HSDMA_CHANNEL_OFFSET(_bchannel, _offset)		\
		(MUSB_HSDMA_BASE + (_bchannel * 0x20) + _offset)

static inline u32 musb_read_hsdma_addr(void __iomem *mbase, u8 bchannel)
{
	u32 addr = musb_readw(mbase,
		MUSB_HSDMA_CHANNEL_OFFSET(bchannel, MUSB_HSDMA_ADDR_HIGH));

	addr = addr << 16;

	addr |= musb_readw(mbase,
		MUSB_HSDMA_CHANNEL_OFFSET(bchannel, MUSB_HSDMA_ADDR_LOW));

	return addr;
}

static inline void musb_write_hsdma_addr(void __iomem *mbase,
				u8 bchannel, dma_addr_t dma_addr)
{
	musb_writew(mbase,
		MUSB_HSDMA_CHANNEL_OFFSET(bchannel, MUSB_HSDMA_ADDR_LOW),
		((u16)((u32) dma_addr & 0xFFFF)));
	musb_writew(mbase,
		MUSB_HSDMA_CHANNEL_OFFSET(bchannel, MUSB_HSDMA_ADDR_HIGH),
		((u16)(((u32) dma_addr >> 16) & 0xFFFF)));
}

static inline u32 musb_read_hsdma_count(void __iomem *mbase, u8 bchannel)
{
	return musb_readl(mbase,
		MUSB_HSDMA_CHANNEL_OFFSET(bchannel, MUSB_HSDMA_COUNT_HIGH));
}

static inline void musb_write_hsdma_count(void __iomem *mbase,
				u8 bchannel, u32 len)
{
	musb_writel(mbase,
		MUSB_HSDMA_CHANNEL_OFFSET(bchannel, MUSB_HSDMA_COUNT_HIGH),
		len);
}

#endif /* CONFIG_BLACKFIN */

/* control register (16-bit): */
#define MUSB_HSDMA_ENABLE_SHIFT		0
#define MUSB_HSDMA_TRANSMIT_SHIFT	1
#define MUSB_HSDMA_MODE1_SHIFT		2
#define MUSB_HSDMA_IRQENABLE_SHIFT	3
#define MUSB_HSDMA_ENDPOINT_SHIFT	4
#define MUSB_HSDMA_BUSERROR_SHIFT	8
#define MUSB_HSDMA_BURSTMODE_SHIFT	9
#define MUSB_HSDMA_BURSTMODE		(3 << MUSB_HSDMA_BURSTMODE_SHIFT)
#define MUSB_HSDMA_BURSTMODE_UNSPEC	0
#define MUSB_HSDMA_BURSTMODE_INCR4	1
#define MUSB_HSDMA_BURSTMODE_INCR8	2
#define MUSB_HSDMA_BURSTMODE_INCR16	3

#define MUSB_HSDMA_CHANNELS		8

struct musb_dma_controller;

struct musb_dma_channel {
	struct dma_channel		channel;
	struct musb_dma_controller	*controller;
	u32				start_addr;
	u32				len;
	u16				max_packet_sz;
	u8				idx;
	u8				epnum;
	u8				transmit;
};

struct musb_dma_controller {
	struct dma_controller		controller;
	struct musb_dma_channel		channel[MUSB_HSDMA_CHANNELS];
	void				*private_data;
	void __iomem			*base;
	u8				channel_count;
	u8				used_channels;
	u8				irq;
};
