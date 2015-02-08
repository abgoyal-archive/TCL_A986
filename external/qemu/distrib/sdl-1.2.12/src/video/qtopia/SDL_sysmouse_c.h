
#include "SDL_config.h"

#include "SDL_lowvideo.h"

/* Functions to be exported */
extern void QT_FreeWMCursor(_THIS, WMcursor *cursor);
extern WMcursor *QT_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);
extern int QT_ShowWMCursor(_THIS, WMcursor *cursor);
extern void QT_WarpWMCursor(_THIS, Uint16 x, Uint16 y);

