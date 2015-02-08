
#include "SDL_config.h"

#include "SDL_x11video.h"

/* Functions to be exported */
extern void X11_SetCaptionNoLock(_THIS, const char *title, const char *icon);
extern void X11_SetCaption(_THIS, const char *title, const char *icon);
extern void X11_SetIcon(_THIS, SDL_Surface *icon, Uint8 *mask);
extern int X11_IconifyWindow(_THIS);
extern SDL_GrabMode X11_GrabInputNoLock(_THIS, SDL_GrabMode mode);
extern SDL_GrabMode X11_GrabInput(_THIS, SDL_GrabMode mode);
extern int X11_GetWMInfo(_THIS, SDL_SysWMinfo *info);
extern void  X11_GetWindowPos(_THIS, int  *px, int  *py);
extern void  X11_SetWindowPos(_THIS, int  x, int  y);
extern int   X11_IsWindowVisible(_THIS, int  recenter);
extern int X11_GetMonitorDPI(_THIS, int* xdpi, int *ydpi);
extern int X11_GetMonitorRect(_THIS, SDL_Rect *rect);
