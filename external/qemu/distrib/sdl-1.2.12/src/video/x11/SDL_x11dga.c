
#include "SDL_config.h"


#include "SDL_video.h"
#include "../SDL_cursor_c.h"
#include "SDL_x11dga_c.h"

/* Global for the error handler */
int dga_event, dga_error = -1;

void X11_EnableDGAMouse(_THIS)
{
#if SDL_VIDEO_DRIVER_X11_DGAMOUSE
    static int use_dgamouse = -1;

    /* Check configuration to see if we should use DGA mouse */
    if ( use_dgamouse < 0 ) {
        int dga_major, dga_minor;
        int dga_flags;
        const char *env_use_dgamouse;

        use_dgamouse = 1;
        env_use_dgamouse = SDL_getenv("SDL_VIDEO_X11_DGAMOUSE");
        if ( env_use_dgamouse ) {
            use_dgamouse = SDL_atoi(env_use_dgamouse);
        }
        /* Check for buggy X servers */
        if ( use_dgamouse && BUGGY_XFREE86(==, 4000) ) {
            use_dgamouse = 0;
        }
        if ( !use_dgamouse || !local_X11 ||
             !SDL_NAME(XF86DGAQueryExtension)(SDL_Display, &dga_event, &dga_error) ||
             !SDL_NAME(XF86DGAQueryVersion)(SDL_Display, &dga_major, &dga_minor) ||
             !SDL_NAME(XF86DGAQueryDirectVideo)(SDL_Display, SDL_Screen, &dga_flags) ||
             !(dga_flags & XF86DGADirectPresent) ) {
            use_dgamouse = 0;
        }
    }

    if ( use_dgamouse && !(using_dga & DGA_MOUSE) ) {
	if ( SDL_NAME(XF86DGADirectVideo)(SDL_Display, SDL_Screen, XF86DGADirectMouse) ) {
            using_dga |= DGA_MOUSE;
        }
    }
#endif /* SDL_VIDEO_DRIVER_X11_DGAMOUSE */
}

/* Argh.  Glide resets DGA mouse mode when it makes the context current! */
void X11_CheckDGAMouse(_THIS)
{
#if SDL_VIDEO_DRIVER_X11_DGAMOUSE
    if ( using_dga & DGA_MOUSE ) {
	SDL_NAME(XF86DGADirectVideo)(SDL_Display,SDL_Screen,XF86DGADirectMouse);
    }
#endif
}

void X11_DisableDGAMouse(_THIS)
{
#if SDL_VIDEO_DRIVER_X11_DGAMOUSE
    if ( using_dga & DGA_MOUSE ) {
	SDL_NAME(XF86DGADirectVideo)(SDL_Display, SDL_Screen, 0);
        using_dga &= ~DGA_MOUSE;
    }
#endif /* SDL_VIDEO_DRIVER_X11_DGAMOUSE */
}
