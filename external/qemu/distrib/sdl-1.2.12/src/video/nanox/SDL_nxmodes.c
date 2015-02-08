
#include "SDL_config.h"

#include "SDL_stdinc.h"
#include "SDL_nxmodes_c.h"

SDL_Rect ** NX_ListModes (_THIS, SDL_PixelFormat * format, Uint32 flags)
{
    if (flags & SDL_FULLSCREEN)
        return SDL_modelist ;

    if (SDL_Visual.bpp == format -> BitsPerPixel) {
        return ((SDL_Rect **) -1) ;
    } else {
        return ((SDL_Rect **) 0) ;
    }
}

void NX_FreeVideoModes (_THIS)
{
    int i ;

    if (SDL_modelist) {
        for (i = 0; SDL_modelist [i]; ++ i) {
            SDL_free (SDL_modelist [i]) ;
        }
        SDL_free (SDL_modelist) ;
        SDL_modelist = NULL;
    }
}

int NX_EnterFullScreen (_THIS)
{
    if (! currently_fullscreen) {
        GR_SCREEN_INFO si ;

        GrGetScreenInfo (& si) ;
        GrResizeWindow (FSwindow, si.cols, si.rows) ;
        GrUnmapWindow (SDL_Window) ;
        GrMapWindow (FSwindow) ;
        GrRaiseWindow (FSwindow) ;
        GrSetFocus (FSwindow) ;
        currently_fullscreen = 1 ;      
    }

    return 1 ;
}

int NX_LeaveFullScreen (_THIS)
{
    if (currently_fullscreen) {
        GrUnmapWindow (FSwindow) ;
        GrMapWindow (SDL_Window) ;
        GrRaiseWindow (SDL_Window) ;
        GrSetFocus (SDL_Window) ;
        currently_fullscreen = 0 ;
    }

    return 0 ;
}
