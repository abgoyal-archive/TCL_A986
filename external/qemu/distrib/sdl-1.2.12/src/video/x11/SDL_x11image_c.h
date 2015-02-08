
#include "SDL_config.h"

#include "SDL_x11video.h"

extern int X11_SetupImage(_THIS, SDL_Surface *screen);
extern void X11_DestroyImage(_THIS, SDL_Surface *screen);
extern int X11_ResizeImage(_THIS, SDL_Surface *screen, Uint32 flags);

extern int X11_AllocHWSurface(_THIS, SDL_Surface *surface);
extern void X11_FreeHWSurface(_THIS, SDL_Surface *surface);
extern int X11_LockHWSurface(_THIS, SDL_Surface *surface);
extern void X11_UnlockHWSurface(_THIS, SDL_Surface *surface);
extern int X11_FlipHWSurface(_THIS, SDL_Surface *surface);

extern void X11_DisableAutoRefresh(_THIS);
extern void X11_EnableAutoRefresh(_THIS);
extern void X11_RefreshDisplay(_THIS);
