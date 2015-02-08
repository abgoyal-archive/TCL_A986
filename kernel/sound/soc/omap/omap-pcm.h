

#ifndef __OMAP_PCM_H__
#define __OMAP_PCM_H__

struct omap_pcm_dma_data {
	char		*name;		/* stream identifier */
	int		dma_req;	/* DMA request line */
	unsigned long	port_addr;	/* transmit/receive register */
	void (*set_threshold)(struct snd_pcm_substream *substream);
	int		data_type;	/* data type 8,16,32 */
	int		sync_mode;	/* DMA sync mode */
	int		packet_size;	/* packet size only in PACKET mode */
};

extern struct snd_soc_platform omap_soc_platform;

#endif
