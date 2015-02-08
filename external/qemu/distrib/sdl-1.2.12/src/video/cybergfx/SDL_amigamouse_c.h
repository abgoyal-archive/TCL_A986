
#include "SDL_config.h"

#include "SDL_cgxvideo.h"

/* Functions to be exported */
extern void amiga_FreeWMCursor(_THIS, WMcursor *cursor);
extern WMcursor *amiga_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);
extern int amiga_ShowWMCursor(_THIS, WMcursor *cursor);
extern void amiga_WarpWMCursor(_THIS, Uint16 x, Uint16 y);
extern void amiga_CheckMouseMode(_THIS);
