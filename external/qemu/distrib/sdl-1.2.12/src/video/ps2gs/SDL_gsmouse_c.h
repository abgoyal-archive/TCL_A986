
#include "SDL_config.h"

#include "SDL_gsvideo.h"

/* This is the maximum size of the cursor sprite */
#define CURSOR_W	32
#define CURSOR_H	32
#define CURSOR_W_POW	5	/* 32 = 2^5 */
#define CURSOR_H_POW	5	/* 32 = 2^5 */

/* Functions to be exported */
extern void GS_FreeWMCursor(_THIS, WMcursor *cursor);
extern WMcursor *GS_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);
extern void GS_MoveWMCursor(_THIS, int x, int y);
extern int GS_ShowWMCursor(_THIS, WMcursor *cursor);
