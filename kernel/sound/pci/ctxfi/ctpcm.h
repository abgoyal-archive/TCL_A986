

#ifndef CTPCM_H
#define CTPCM_H

#include "ctatc.h"

int ct_alsa_pcm_create(struct ct_atc *atc,
		       enum CTALSADEVS device,
		       const char *device_name);

#endif /* CTPCM_H */
