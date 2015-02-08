
#include "SDL_config.h"

#include "SDL_aavideo.h"

extern void AA_initkeymaps(int fd);
extern void AA_mousecallback(int button, int dx, int dy,
                                 int u1,int u2,int u3, int u4);
extern void AA_keyboardcallback(int scancode, int pressed);

extern void AA_InitOSKeymap(_THIS);
extern void AA_PumpEvents(_THIS);
