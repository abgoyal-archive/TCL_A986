
#include "SDL_config.h"

#include "SDL_nxvideo.h"

extern int NX_SetupImage (_THIS, SDL_Surface * screen) ;
extern void NX_DestroyImage (_THIS, SDL_Surface * screen) ;
extern int NX_ResizeImage (_THIS, SDL_Surface * screen, Uint32 flags) ;

extern void NX_NormalUpdate (_THIS, int numrects, SDL_Rect * rects) ;
extern void NX_RefreshDisplay (_THIS) ;
