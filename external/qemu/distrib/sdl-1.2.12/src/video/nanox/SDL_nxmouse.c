
#include "SDL_config.h"

#include "../../events/SDL_events_c.h"

#include "SDL_nxmouse_c.h"

// The implementation dependent data for the window manager cursor
struct WMcursor {
    int unused ;
} ;

WMcursor * NX_CreateWMCursor (_THIS,
        Uint8 * data, Uint8 * mask, int w, int h, int hot_x, int hot_y)
{
    WMcursor * cursor ;

    Dprintf ("enter NX_CreateWMCursor\n") ;

    cursor = (WMcursor *) SDL_malloc (sizeof (WMcursor)) ;
    if (cursor == NULL) {
        SDL_OutOfMemory () ;
        return NULL ;
    }

    Dprintf ("leave NX_CreateWMCursor\n") ;
    return cursor ;
}

void NX_FreeWMCursor (_THIS, WMcursor * cursor)
{
    Dprintf ("NX_FreeWMCursor\n") ;
    SDL_free (cursor) ;
    return ;
}

void NX_WarpWMCursor(_THIS, Uint16 x, Uint16 y)
{
    GR_WINDOW_INFO info ;

    Dprintf ("enter NX_WarpWMCursor\n") ;
    SDL_Lock_EventThread () ;
    
    GrGetWindowInfo (SDL_Window, & info) ;
    GrMoveCursor (info.x + x, info.y + y) ;

    SDL_Unlock_EventThread () ;
    Dprintf ("leave NX_WarpWMCursor\n") ;
}

int NX_ShowWMCursor (_THIS, WMcursor * cursor)
{
    Dprintf ("NX_ShowWMCursor\n") ;
    return 1 ;
}
