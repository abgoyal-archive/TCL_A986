
#include "SDL_config.h"

#include "SDL_riscosvideo.h"

/* The implementation dependent data for the window manager cursor */
struct WMcursor {
	int w;
	int h;
	int hot_x;
	int hot_y;
	Uint8 *data;
};

/* Functions to be exported */
void RISCOS_FreeWMCursor(_THIS, WMcursor *cursor);
WMcursor *RISCOS_CreateWMCursor(_THIS, Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);

int RISCOS_ShowWMCursor(_THIS, WMcursor *cursor);
void FULLSCREEN_WarpWMCursor(_THIS, Uint16 x, Uint16 y);

int WIMP_ShowWMCursor(_THIS, WMcursor *cursor);
void WIMP_WarpWMCursor(_THIS, Uint16 x, Uint16 y);

