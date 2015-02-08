
#include "SDL_config.h"

#include "SDL_vglvideo.h"

/* Functions to be exported */
extern void VGL_FreeWMCursor(_THIS, WMcursor *cursor);
extern WMcursor *VGL_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);
extern int VGL_ShowWMCursor(_THIS, WMcursor *cursor);
extern void VGL_WarpWMCursor(_THIS, Uint16 x, Uint16 y);

