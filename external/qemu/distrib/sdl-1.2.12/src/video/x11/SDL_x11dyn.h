
#include "SDL_config.h"

#ifndef _SDL_x11dyn_h
#define _SDL_x11dyn_h

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>
#include <X11/Xlibint.h>
#include <X11/Xproto.h>

#include "../Xext/extensions/Xext.h"
#include "../Xext/extensions/extutil.h"

#ifndef NO_SHARED_MEMORY
#include <sys/ipc.h>
#include <sys/shm.h>
#include <X11/extensions/XShm.h>
#endif

#if SDL_VIDEO_DRIVER_X11_XRANDR
#include <X11/extensions/Xrandr.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* evil function signatures... */
typedef Bool (*SDL_X11_XESetWireToEventRetType)(Display*,XEvent*,xEvent*);
typedef int (*SDL_X11_XSynchronizeRetType)(Display*);
typedef Status (*SDL_X11_XESetEventToWireRetType)(Display*,XEvent*,xEvent*);

int SDL_X11_LoadSymbols(void);
void SDL_X11_UnloadSymbols(void);

/* That's really annoying...make this a function pointer no matter what. */
#ifdef X_HAVE_UTF8_STRING
extern XIC (*pXCreateIC)(XIM,...);
extern char *(*pXGetICValues)(XIC, ...);
#endif

/* These SDL_X11_HAVE_* flags are here whether you have dynamic X11 or not. */
#define SDL_X11_MODULE(modname) extern int SDL_X11_HAVE_##modname;
#define SDL_X11_SYM(rc,fn,params,args,ret)
#include "SDL_x11sym.h"
#undef SDL_X11_MODULE
#undef SDL_X11_SYM


#ifdef __cplusplus
}
#endif

#endif  /* !defined _SDL_x11dyn_h */

