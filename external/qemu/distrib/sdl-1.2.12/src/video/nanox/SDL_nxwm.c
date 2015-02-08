
#include "SDL_config.h"

#include "SDL_syswm.h"
#include "../../events/SDL_events_c.h"

#include "SDL_nxwm_c.h"

void NX_SetCaption (_THIS, const char * title, const char * icon)
{
    Dprintf ("enter NX_SetCaption\n") ;

    // Lock the event thread, in multi-threading environments
    SDL_Lock_EventThread () ;
    
    if (SDL_Window) 
        GrSetWindowTitle (SDL_Window, title) ;
    
    SDL_Unlock_EventThread () ;
    Dprintf ("leave NX_SetCaption\n") ;
}

int NX_GetWMInfo (_THIS, SDL_SysWMinfo * info)
{
    Dprintf ("enter NX_GetWMInfo\n") ;

    if (info -> version.major <= SDL_MAJOR_VERSION) {
        info -> window = SDL_Window ;
        return 1 ;
    } else {
        SDL_SetError("Application not compiled with SDL %d.%d\n",
            SDL_MAJOR_VERSION, SDL_MINOR_VERSION) ;
        return -1 ;
    }

    Dprintf ("leave NX_GetWMInfo\n") ;
}
