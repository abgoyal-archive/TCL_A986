
#include "SDL_config.h"

#include "SDL_lowvideo.h"

/* Functions to be exported */
extern void BE_FreeWMCursor(_THIS, WMcursor *cursor);
extern WMcursor *BE_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);
extern int BE_ShowWMCursor(_THIS, WMcursor *cursor);
extern void BE_WarpWMCursor(_THIS, Uint16 x, Uint16 y);
extern void BE_CheckMouseMode(_THIS);

