

#ifndef __LINUX_SND_SOC_PCM3008_H
#define __LINUX_SND_SOC_PCM3008_H

struct pcm3008_setup_data {
	unsigned dem0_pin;
	unsigned dem1_pin;
	unsigned pdad_pin;
	unsigned pdda_pin;
};

extern struct snd_soc_codec_device soc_codec_dev_pcm3008;
extern struct snd_soc_dai pcm3008_dai;

#endif
