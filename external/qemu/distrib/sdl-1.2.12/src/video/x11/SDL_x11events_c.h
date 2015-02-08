
#include "SDL_config.h"

#include "SDL_x11video.h"

/* Functions to be exported */
extern void X11_InitOSKeymap(_THIS);
extern void X11_PumpEvents(_THIS);
extern void X11_SetKeyboardState(Display *display, const char *key_vec);

extern void X11_SaveScreenSaver(Display *display, int *saved_timeout, BOOL *dpms);
extern void X11_DisableScreenSaver(_THIS, Display *display);
extern void X11_RestoreScreenSaver(_THIS, Display *display, int saved_timeout, BOOL dpms);
