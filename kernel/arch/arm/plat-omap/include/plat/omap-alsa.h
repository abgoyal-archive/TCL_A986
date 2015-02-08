

#ifndef __OMAP_ALSA_H
#define __OMAP_ALSA_H

#include <plat/dma.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <plat/mcbsp.h>
#include <linux/platform_device.h>

#define DMA_BUF_SIZE	(1024 * 8)

struct audio_stream {
	char *id;		/* identification string */
	int stream_id;		/* numeric identification */
	int dma_dev;		/* dma number of that device */
	int *lch;		/* Chain of channels this stream is linked to */
	char started;		/* to store if the chain was started or not */
	int dma_q_head;		/* DMA Channel Q Head */
	int dma_q_tail;		/* DMA Channel Q Tail */
	char dma_q_count;	/* DMA Channel Q Count */
	int active:1;		/* we are using this stream for transfer now */
	int period;		/* current transfer period */
	int periods;		/* current count of periods registerd in the DMA engine */
	spinlock_t dma_lock;	/* for locking in DMA operations */
	struct snd_pcm_substream *stream;	/* the pcm stream */
	unsigned linked:1;	/* dma channels linked */
	int offset;		/* store start position of the last period in the alsa buffer */
	int (*hw_start)(void);  /* interface to start HW interface, e.g. McBSP */
	int (*hw_stop)(void);   /* interface to stop HW interface, e.g. McBSP */
};

struct snd_card_omap_codec {
	struct snd_card *card;
	struct snd_pcm *pcm;
	long samplerate;
	struct audio_stream s[2];	/* playback & capture */
};

struct omap_alsa_codec_config {
	char 	*name;
	struct	omap_mcbsp_reg_cfg *mcbsp_regs_alsa;
	struct	snd_pcm_hw_constraint_list *hw_constraints_rates;
	struct	snd_pcm_hardware *snd_omap_alsa_playback;
	struct	snd_pcm_hardware *snd_omap_alsa_capture;
	void	(*codec_configure_dev)(void);
	void	(*codec_set_samplerate)(long);
	void	(*codec_clock_setup)(void);
	int	(*codec_clock_on)(void);
	int 	(*codec_clock_off)(void);
	int	(*get_default_samplerate)(void);
};

/*********** Mixer function prototypes *************************/
int snd_omap_mixer(struct snd_card_omap_codec *);
void snd_omap_init_mixer(void);

#ifdef CONFIG_PM
void snd_omap_suspend_mixer(void);
void snd_omap_resume_mixer(void);
#endif

int snd_omap_alsa_post_probe(struct platform_device *pdev, struct omap_alsa_codec_config *config);
int snd_omap_alsa_remove(struct platform_device *pdev);
#ifdef CONFIG_PM
int snd_omap_alsa_suspend(struct platform_device *pdev, pm_message_t state);
int snd_omap_alsa_resume(struct platform_device *pdev);
#else
#define snd_omap_alsa_suspend	NULL
#define snd_omap_alsa_resume	NULL
#endif

void callback_omap_alsa_sound_dma(void *);

#endif
