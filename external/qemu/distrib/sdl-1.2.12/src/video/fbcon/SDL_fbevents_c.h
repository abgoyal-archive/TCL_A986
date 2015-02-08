
#include "SDL_config.h"

#include "SDL_fbvideo.h"

extern int FB_OpenKeyboard(_THIS);
extern void FB_CloseKeyboard(_THIS);
extern int FB_OpenMouse(_THIS);
extern void FB_CloseMouse(_THIS);
extern int FB_EnterGraphicsMode(_THIS);
extern int FB_InGraphicsMode(_THIS);
extern void FB_LeaveGraphicsMode(_THIS);

extern void FB_InitOSKeymap(_THIS);
extern void FB_PumpEvents(_THIS);
