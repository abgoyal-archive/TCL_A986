

#ifndef __SOUND_HDA_BEEP_H
#define __SOUND_HDA_BEEP_H

#include "hda_codec.h"

#define HDA_BEEP_MODE_OFF	0
#define HDA_BEEP_MODE_ON	1
#define HDA_BEEP_MODE_SWREG	2

/* beep information */
struct hda_beep {
	struct input_dev *dev;
	struct hda_codec *codec;
	unsigned int mode;
	char phys[32];
	int tone;
	hda_nid_t nid;
	unsigned int enabled:1;
	unsigned int request_enable:1;
	unsigned int linear_tone:1;	/* linear tone for IDT/STAC codec */
	struct work_struct register_work; /* registration work */
	struct delayed_work unregister_work; /* unregistration work */
	struct work_struct beep_work; /* scheduled task for beep event */
	struct mutex mutex;
};

#ifdef CONFIG_SND_HDA_INPUT_BEEP
int snd_hda_enable_beep_device(struct hda_codec *codec, int enable);
int snd_hda_attach_beep_device(struct hda_codec *codec, int nid);
void snd_hda_detach_beep_device(struct hda_codec *codec);
#else
#define snd_hda_attach_beep_device(...)		0
#define snd_hda_detach_beep_device(...)
#endif
#endif
