
#include "SDL_config.h"

#include "SDL_nxvideo.h"
#include <SDL.h>

extern SDL_Rect ** NX_ListModes (_THIS, SDL_PixelFormat * format, Uint32 flags) ;
extern void NX_FreeVideoModes (_THIS) ;
extern int NX_EnterFullScreen (_THIS) ;
extern int NX_LeaveFullScreen (_THIS) ;
