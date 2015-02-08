

#ifndef _DAVINCI_PCM_H
#define _DAVINCI_PCM_H

#include <mach/edma.h>
#include <mach/asp.h>


struct davinci_pcm_dma_params {
	int channel;			/* sync dma channel ID */
	unsigned short acnt;
	dma_addr_t dma_addr;		/* device physical address for DMA */
	unsigned sram_size;
	enum dma_event_q eventq_no;	/* event queue number */
	unsigned char data_type;	/* xfer data type */
	unsigned char convert_mono_stereo;
	unsigned int fifo_level;
};


extern struct snd_soc_platform davinci_soc_platform;

#endif
