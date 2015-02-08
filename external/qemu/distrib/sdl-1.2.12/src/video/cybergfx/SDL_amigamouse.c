
#include "SDL_config.h"

#include "SDL_mouse.h"
#include "../../events/SDL_events_c.h"
#include "../SDL_cursor_c.h"
#include "SDL_amigamouse_c.h"


/* The implementation dependent data for the window manager cursor */

typedef void * WMCursor;

void amiga_FreeWMCursor(_THIS, WMcursor *cursor)
{
}

WMcursor *amiga_CreateWMCursor(_THIS,
		Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y)
{
	return (WMcursor *)1; // Amiga has an Hardware cursor, so it's ok to return something unuseful but true
}

int amiga_ShowWMCursor(_THIS, WMcursor *cursor)
{
	/* Don't do anything if the display is gone */
	if ( SDL_Display == NULL) {
		return(0);
	}

	/* Set the Amiga prefs cursor cursor, or blank if cursor is NULL */

	if ( SDL_Window ) {
		SDL_Lock_EventThread();
		if ( cursor == NULL ) {
			if ( SDL_BlankCursor != NULL ) {
// Hide cursor HERE
				SetPointer(SDL_Window,(UWORD *)SDL_BlankCursor,1,1,0,0);
			}
		} else {
// Show cursor
			ClearPointer(SDL_Window);
		}
		SDL_Unlock_EventThread();
	}
	return(1);
}

void amiga_WarpWMCursor(_THIS, Uint16 x, Uint16 y)
{
/* FIXME: Not implemented */
}

/* Check to see if we need to enter or leave mouse relative mode */
void amiga_CheckMouseMode(_THIS)
{
}
