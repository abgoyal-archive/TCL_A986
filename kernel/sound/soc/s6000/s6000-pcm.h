

#ifndef _S6000_PCM_H
#define _S6000_PCM_H

struct snd_soc_dai;
struct snd_pcm_substream;

struct s6000_pcm_dma_params {
	unsigned int (*check_xrun)(struct snd_soc_dai *cpu_dai);
	int (*trigger)(struct snd_pcm_substream *substream, int cmd, int after);
	dma_addr_t sif_in;
	dma_addr_t sif_out;
	u32 dma_in;
	u32 dma_out;
	int irq;
	int same_rate;

	spinlock_t lock;
	int in_use;
	int rate;
};

extern struct snd_soc_platform s6000_soc_platform;

#endif
