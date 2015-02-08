
#include "SDL_config.h"

#include "SDL_cgxvideo.h"

extern int CGX_SetupImage(_THIS, SDL_Surface *screen);
extern void CGX_DestroyImage(_THIS, SDL_Surface *screen);
extern int CGX_ResizeImage(_THIS, SDL_Surface *screen, Uint32 flags);

extern int CGX_AllocHWSurface(_THIS, SDL_Surface *surface);
extern void CGX_FreeHWSurface(_THIS, SDL_Surface *surface);
extern int CGX_LockHWSurface(_THIS, SDL_Surface *surface);
extern void CGX_UnlockHWSurface(_THIS, SDL_Surface *surface);
extern int CGX_FlipHWSurface(_THIS, SDL_Surface *surface);

extern void CGX_RefreshDisplay(_THIS);
