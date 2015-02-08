
#include "SDL_config.h"

/* Functions and variables exported from SDL_audio.c for SDL_sysaudio.c */

/* Functions to get a list of "close" audio formats */
extern Uint16 SDL_FirstAudioFormat(Uint16 format);
extern Uint16 SDL_NextAudioFormat(void);

/* Function to calculate the size and silence for a SDL_AudioSpec */
extern void SDL_CalculateAudioSpec(SDL_AudioSpec *spec);

/* The actual mixing thread function */
extern int SDLCALL SDL_RunAudio(void *audiop);
