
#include "SDL_config.h"

#include "../macrom/SDL_romvideo.h"

/* Functions to be exported */
extern void Mac_FreeWMCursor(_THIS, WMcursor *cursor);
extern WMcursor *Mac_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);
extern int Mac_ShowWMCursor(_THIS, WMcursor *cursor);
extern void Mac_WarpWMCursor(_THIS, Uint16 x, Uint16 y);

/* Data to be exported */
extern int Mac_cursor_showing;
