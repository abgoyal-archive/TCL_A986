

#ifndef CAPTURE_H
#define CAPTURE_H


#include "driver.h"

#include <sound/pcm.h>

#include "pcm.h"


extern struct snd_pcm_ops snd_line6_capture_ops;


extern int create_audio_in_urbs(struct snd_line6_pcm *line6pcm);
extern int snd_line6_capture_trigger(struct snd_pcm_substream *substream,
				     int cmd);
extern void unlink_wait_clear_audio_in_urbs(struct snd_line6_pcm *line6pcm);


#endif
