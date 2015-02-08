
#include "SDL_config.h"

#include "SDL_lowvideo.h"

/* Functions to be exported */
extern void WIN_FreeWMCursor(_THIS, WMcursor *cursor);
extern WMcursor *WIN_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);
extern int WIN_ShowWMCursor(_THIS, WMcursor *cursor);
extern void WIN_WarpWMCursor(_THIS, Uint16 x, Uint16 y);
extern void WIN_UpdateMouse(_THIS);
extern void WIN_CheckMouseMode(_THIS);
