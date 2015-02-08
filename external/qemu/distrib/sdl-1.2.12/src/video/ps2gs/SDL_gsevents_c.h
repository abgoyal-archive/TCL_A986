
#include "SDL_config.h"

#include "SDL_gsvideo.h"

extern int GS_OpenKeyboard(_THIS);
extern void GS_CloseKeyboard(_THIS);
extern int GS_OpenMouse(_THIS);
extern void GS_CloseMouse(_THIS);
extern int GS_EnterGraphicsMode(_THIS);
extern int GS_InGraphicsMode(_THIS);
extern void GS_LeaveGraphicsMode(_THIS);

extern void GS_InitOSKeymap(_THIS);
extern void GS_PumpEvents(_THIS);
