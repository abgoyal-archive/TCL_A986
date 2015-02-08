
#include "SDL_config.h"


#ifndef _SDL_epocvideo_h
#define _SDL_epocvideo_h

extern "C" {
#include "SDL_mouse.h"
#include "../SDL_sysvideo.h"
};

#include <e32std.h>
#include <bitdev.h> 
#include <w32std.h>

/* Hidden "this" pointer for the video functions */
#define _THIS	SDL_VideoDevice *_this
#define Private	_this->hidden

#define SDL_NUMMODES	4

/* Private display data */
struct SDL_PrivateVideoData {

    SDL_Rect            *SDL_modelist[SDL_NUMMODES+1];

	/* Epoc window server info */
    
    RWsSession			EPOC_WsSession;
	RWindowGroup		EPOC_WsWindowGroup;
    TInt                EPOC_WsWindowGroupID;
	RWindow				EPOC_WsWindow;
	CWsScreenDevice*	EPOC_WsScreen;
	CWindowGc*			EPOC_WindowGc;
	TRequestStatus		EPOC_WsEventStatus;
	TRequestStatus		EPOC_RedrawEventStatus;
	TWsEvent			EPOC_WsEvent;
	TWsRedrawEvent		EPOC_RedrawEvent;
    #ifdef __WINS__
    CWsBitmap*          EPOC_Bitmap;
    #endif
    TBool               EPOC_IsWindowFocused; //!!Not used for anything yet!

    /* Screen hardware frame buffer info */

   	TBool				EPOC_HasFrameBuffer;
	TInt				EPOC_BytesPerPixel;
	TInt				EPOC_BytesPerScanLine;
	TDisplayMode		EPOC_DisplayMode;
	TSize				EPOC_ScreenSize;
	TUint8*				EPOC_FrameBuffer;		/* if NULL in HW we can't do direct screen access */
    TInt                EPOC_ScreenOffset;

    /* Simulate double screen height */
    TBool               EPOC_ShrinkedHeight;
};

extern "C" {
extern void RedrawWindowL(_THIS);
};


#endif /* _SDL_epocvideo_h */
