


#ifndef STE_DMA40_H
#define STE_DMA40_H

#include <linux/dmaengine.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>

/* dev types for memcpy */
#define STEDMA40_DEV_DST_MEMORY (-1)
#define	STEDMA40_DEV_SRC_MEMORY (-1)


/* Priority */
#define STEDMA40_INFO_PRIO_TYPE_POS 2
#define STEDMA40_HIGH_PRIORITY_CHANNEL (0x1 << STEDMA40_INFO_PRIO_TYPE_POS)
#define STEDMA40_LOW_PRIORITY_CHANNEL (0x2 << STEDMA40_INFO_PRIO_TYPE_POS)

/* Mode  */
#define STEDMA40_INFO_CH_MODE_TYPE_POS 6
#define STEDMA40_CHANNEL_IN_PHY_MODE (0x1 << STEDMA40_INFO_CH_MODE_TYPE_POS)
#define STEDMA40_CHANNEL_IN_LOG_MODE (0x2 << STEDMA40_INFO_CH_MODE_TYPE_POS)
#define STEDMA40_CHANNEL_IN_OPER_MODE (0x3 << STEDMA40_INFO_CH_MODE_TYPE_POS)

/* Mode options */
#define STEDMA40_INFO_CH_MODE_OPT_POS 8
#define STEDMA40_PCHAN_BASIC_MODE (0x1 << STEDMA40_INFO_CH_MODE_OPT_POS)
#define STEDMA40_PCHAN_MODULO_MODE (0x2 << STEDMA40_INFO_CH_MODE_OPT_POS)
#define STEDMA40_PCHAN_DOUBLE_DST_MODE (0x3 << STEDMA40_INFO_CH_MODE_OPT_POS)
#define STEDMA40_LCHAN_SRC_PHY_DST_LOG (0x1 << STEDMA40_INFO_CH_MODE_OPT_POS)
#define STEDMA40_LCHAN_SRC_LOG_DST_PHS (0x2 << STEDMA40_INFO_CH_MODE_OPT_POS)
#define STEDMA40_LCHAN_SRC_LOG_DST_LOG (0x3 << STEDMA40_INFO_CH_MODE_OPT_POS)

/* Interrupt */
#define STEDMA40_INFO_TIM_POS 10
#define STEDMA40_NO_TIM_FOR_LINK (0x0 << STEDMA40_INFO_TIM_POS)
#define STEDMA40_TIM_FOR_LINK (0x1 << STEDMA40_INFO_TIM_POS)

/* End of channel_type configuration */

#define STEDMA40_ESIZE_8_BIT  0x0
#define STEDMA40_ESIZE_16_BIT 0x1
#define STEDMA40_ESIZE_32_BIT 0x2
#define STEDMA40_ESIZE_64_BIT 0x3

/* The value 4 indicates that PEN-reg shall be set to 0 */
#define STEDMA40_PSIZE_PHY_1  0x4
#define STEDMA40_PSIZE_PHY_2  0x0
#define STEDMA40_PSIZE_PHY_4  0x1
#define STEDMA40_PSIZE_PHY_8  0x2
#define STEDMA40_PSIZE_PHY_16 0x3

#define STEDMA40_PSIZE_LOG_1  STEDMA40_PSIZE_PHY_2
#define STEDMA40_PSIZE_LOG_4  STEDMA40_PSIZE_PHY_4
#define STEDMA40_PSIZE_LOG_8  STEDMA40_PSIZE_PHY_8
#define STEDMA40_PSIZE_LOG_16 STEDMA40_PSIZE_PHY_16

enum stedma40_flow_ctrl {
	STEDMA40_NO_FLOW_CTRL,
	STEDMA40_FLOW_CTRL,
};

enum stedma40_endianess {
	STEDMA40_LITTLE_ENDIAN,
	STEDMA40_BIG_ENDIAN
};

enum stedma40_periph_data_width {
	STEDMA40_BYTE_WIDTH = STEDMA40_ESIZE_8_BIT,
	STEDMA40_HALFWORD_WIDTH = STEDMA40_ESIZE_16_BIT,
	STEDMA40_WORD_WIDTH = STEDMA40_ESIZE_32_BIT,
	STEDMA40_DOUBLEWORD_WIDTH = STEDMA40_ESIZE_64_BIT
};

struct stedma40_half_channel_info {
	enum stedma40_endianess endianess;
	enum stedma40_periph_data_width data_width;
	int psize;
	enum stedma40_flow_ctrl flow_ctrl;
};

enum stedma40_xfer_dir {
	STEDMA40_MEM_TO_MEM,
	STEDMA40_MEM_TO_PERIPH,
	STEDMA40_PERIPH_TO_MEM,
	STEDMA40_PERIPH_TO_PERIPH
};


struct stedma40_chan_cfg {
	enum stedma40_xfer_dir			 dir;
	unsigned int				 channel_type;
	int					 src_dev_type;
	int					 dst_dev_type;
	struct stedma40_half_channel_info	 src_info;
	struct stedma40_half_channel_info	 dst_info;
	void					*pre_transfer_data;
	int (*pre_transfer)			(struct dma_chan *chan,
						 void *data,
						 int size);
};

struct stedma40_platform_data {
	u32				 dev_len;
	const dma_addr_t		*dev_tx;
	const dma_addr_t		*dev_rx;
	int				*memcpy;
	u32				 memcpy_len;
	struct stedma40_chan_cfg	*memcpy_conf_phy;
	struct stedma40_chan_cfg	*memcpy_conf_log;
	unsigned int			 llis_per_log;
};

int stedma40_set_psize(struct dma_chan *chan,
		       int src_psize,
		       int dst_psize);


bool stedma40_filter(struct dma_chan *chan, void *data);


struct dma_async_tx_descriptor *stedma40_memcpy_sg(struct dma_chan *chan,
						   struct scatterlist *sgl_dst,
						   struct scatterlist *sgl_src,
						   unsigned int sgl_len,
						   unsigned long flags);


static inline struct
dma_async_tx_descriptor *stedma40_slave_mem(struct dma_chan *chan,
					    dma_addr_t addr,
					    unsigned int size,
					    enum dma_data_direction direction,
					    unsigned long flags)
{
	struct scatterlist sg;
	sg_init_table(&sg, 1);
	sg.dma_address = addr;
	sg.length = size;

	return chan->device->device_prep_slave_sg(chan, &sg, 1,
						  direction, flags);
}

#endif
