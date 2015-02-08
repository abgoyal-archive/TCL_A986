
#include "SDL_config.h"

#include "SDL_svgavideo.h"

extern int SVGA_initkeymaps(int fd);
extern void SVGA_mousecallback(int button, int dx, int dy,
                                 int u1,int u2,int u3, int u4);
extern void SVGA_keyboardcallback(int scancode, int pressed);

extern void SVGA_InitOSKeymap(_THIS);
extern void SVGA_PumpEvents(_THIS);
