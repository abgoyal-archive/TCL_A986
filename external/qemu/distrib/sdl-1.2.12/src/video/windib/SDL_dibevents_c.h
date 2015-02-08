
#include "SDL_config.h"

#include "../wincommon/SDL_lowvideo.h"

extern LONG
 DIB_HandleMessage(_THIS, HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
extern int DIB_CreateWindow(_THIS);
extern void DIB_DestroyWindow(_THIS);

extern void DIB_PumpEvents(_THIS);
extern void DIB_InitOSKeymap(_THIS);
