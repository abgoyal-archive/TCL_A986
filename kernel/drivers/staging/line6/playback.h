

#ifndef PLAYBACK_H
#define PLAYBACK_H


#include "driver.h"

#include <sound/pcm.h>


extern struct snd_pcm_ops snd_line6_playback_ops;


extern int create_audio_out_urbs(struct snd_line6_pcm *line6pcm);
extern int snd_line6_playback_trigger(struct snd_pcm_substream *substream,
				      int cmd);
extern void unlink_wait_clear_audio_out_urbs(struct snd_line6_pcm *line6pcm);


#endif
