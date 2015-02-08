
#include "SDL_config.h"


#if defined(__M68000__) && defined(__GNUC__)
void SDL_MixAudio_m68k_U8(char* dst,char* src, long len, long volume, char* mix8);
void SDL_MixAudio_m68k_S8(char* dst,char* src, long len, long volume);

void SDL_MixAudio_m68k_S16MSB(short* dst,short* src, long len, long volume);
void SDL_MixAudio_m68k_S16LSB(short* dst,short* src, long len, long volume);
#endif
