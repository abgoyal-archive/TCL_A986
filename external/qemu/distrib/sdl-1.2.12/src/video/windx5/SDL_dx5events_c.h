
#include "SDL_config.h"

#include "../wincommon/SDL_lowvideo.h"

extern LONG
 DX5_HandleMessage(_THIS, HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam);
extern int DX5_CreateWindow(_THIS);
extern void DX5_DestroyWindow(_THIS);

extern void DX5_PumpEvents(_THIS);
extern void DX5_InitOSKeymap(_THIS);
extern void DX5_DInputReset(_THIS, int fullscreen);

