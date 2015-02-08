
#include "SDL_config.h"


#if ((defined(_MSC_VER) && defined(_M_IX86)) || defined(__WATCOMC__)) && defined(SDL_ASSEMBLY_ROUTINES)
void SDL_MixAudio_MMX_S16_VC(char* ,char* ,unsigned int ,int );
void SDL_MixAudio_MMX_S8_VC(char* ,char* ,unsigned int ,int );
#endif
