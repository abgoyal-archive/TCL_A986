

/* Include file for SDL mouse event handling */

#ifndef _SDL_mouse_h
#define _SDL_mouse_h

#include "SDL_stdinc.h"
#include "SDL_error.h"
#include "SDL_video.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

typedef struct WMcursor WMcursor;	/* Implementation dependent */
typedef struct SDL_Cursor {
	SDL_Rect area;			/* The area of the mouse cursor */
	Sint16 hot_x, hot_y;		/* The "tip" of the cursor */
	Uint8 *data;			/* B/W cursor data */
	Uint8 *mask;			/* B/W cursor mask */
	Uint8 *save[2];			/* Place to save cursor area */
	WMcursor *wm_cursor;		/* Window-manager cursor */
} SDL_Cursor;

/* Function prototypes */
extern DECLSPEC Uint8 SDLCALL SDL_GetMouseState(int *x, int *y);

extern DECLSPEC Uint8 SDLCALL SDL_GetRelativeMouseState(int *x, int *y);

extern DECLSPEC void SDLCALL SDL_WarpMouse(Uint16 x, Uint16 y);

extern DECLSPEC SDL_Cursor * SDLCALL SDL_CreateCursor
		(Uint8 *data, Uint8 *mask, int w, int h, int hot_x, int hot_y);

extern DECLSPEC void SDLCALL SDL_SetCursor(SDL_Cursor *cursor);

extern DECLSPEC SDL_Cursor * SDLCALL SDL_GetCursor(void);

extern DECLSPEC void SDLCALL SDL_FreeCursor(SDL_Cursor *cursor);

extern DECLSPEC int SDLCALL SDL_ShowCursor(int toggle);

#define SDL_BUTTON(X)		(1 << ((X)-1))
#define SDL_BUTTON_LEFT		1
#define SDL_BUTTON_MIDDLE	2
#define SDL_BUTTON_RIGHT	3
#define SDL_BUTTON_WHEELUP	4
#define SDL_BUTTON_WHEELDOWN	5
#define SDL_BUTTON_LMASK	SDL_BUTTON(SDL_BUTTON_LEFT)
#define SDL_BUTTON_MMASK	SDL_BUTTON(SDL_BUTTON_MIDDLE)
#define SDL_BUTTON_RMASK	SDL_BUTTON(SDL_BUTTON_RIGHT)


/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_mouse_h */
