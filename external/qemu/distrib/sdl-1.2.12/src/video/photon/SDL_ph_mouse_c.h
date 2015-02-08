
#include "SDL_config.h"

#ifndef __SDL_PH_MOUSE_H__
#define __SDL_PH_MOUSE_H__

#include "SDL_ph_video.h"

/* Functions to be exported */
extern void ph_FreeWMCursor(_THIS, WMcursor *cursor);
extern WMcursor *ph_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);
extern PhCursorDef_t ph_GetWMPhCursor(WMcursor *cursor);
extern int ph_ShowWMCursor(_THIS, WMcursor *cursor);
extern void ph_WarpWMCursor(_THIS, Uint16 x, Uint16 y);
extern void ph_CheckMouseMode(_THIS);
extern void ph_UpdateMouse(_THIS);

#endif /* __SDL_PH_MOUSE_H__ */
