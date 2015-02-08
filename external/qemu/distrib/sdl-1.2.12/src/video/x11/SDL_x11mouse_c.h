
#include "SDL_config.h"

#include "SDL_x11video.h"

/* Functions to be exported */
extern void X11_FreeWMCursor(_THIS, WMcursor *cursor);
extern WMcursor *X11_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);
extern int X11_ShowWMCursor(_THIS, WMcursor *cursor);
extern void X11_WarpWMCursor(_THIS, Uint16 x, Uint16 y);
extern void X11_CheckMouseModeNoLock(_THIS);
extern void X11_CheckMouseMode(_THIS);
