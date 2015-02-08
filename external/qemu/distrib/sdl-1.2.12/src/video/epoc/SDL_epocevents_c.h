
#include "SDL_config.h"



extern "C" {
#include "../SDL_sysvideo.h"
};

#define MAX_SCANCODE 255


extern "C" {
extern void EPOC_InitOSKeymap(_THIS);
extern void EPOC_PumpEvents(_THIS);
};

