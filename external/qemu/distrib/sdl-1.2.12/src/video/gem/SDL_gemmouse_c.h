
#include "SDL_config.h"

#include "SDL_gemvideo.h"

/* Functions to be exported */
extern void GEM_FreeWMCursor(_THIS, WMcursor *cursor);
extern WMcursor *GEM_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);
extern int GEM_ShowWMCursor(_THIS, WMcursor *cursor);
#if 0
extern void GEM_WarpWMCursor(_THIS, Uint16 x, Uint16 y);
#endif
extern void GEM_CheckMouseMode(_THIS);
