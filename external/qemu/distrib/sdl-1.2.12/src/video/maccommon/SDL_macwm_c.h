
#include "SDL_config.h"

#include "../macrom/SDL_romvideo.h"

/* Functions to be exported */
extern void Mac_SetCaption(_THIS, const char *title, const char *icon);

#define SDL_MACCLASSIC_GAMMA_SUPPORT ((defined(__APPLE__) && defined(__MACH__)) == 0)

#if SDL_MACCLASSIC_GAMMA_SUPPORT
extern void Mac_QuitGamma(_THIS);
extern int Mac_SetGammaRamp(_THIS, Uint16 *ramp);
extern int Mac_GetGammaRamp(_THIS, Uint16 *ramp);
#endif

