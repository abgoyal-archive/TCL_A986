
#include "SDL_config.h"

/* Useful functions and variables from SDL_events.c */
#include "SDL_events.h"

/* Start and stop the event processing loop */
extern int SDL_StartEventLoop(Uint32 flags);
extern void SDL_StopEventLoop(void);
extern void SDL_QuitInterrupt(void);

extern void SDL_Lock_EventThread(void);
extern void SDL_Unlock_EventThread(void);
extern Uint32 SDL_EventThreadID(void);

/* Event handler init routines */
extern int  SDL_AppActiveInit(void);
extern int  SDL_KeyboardInit(void);
extern int  SDL_MouseInit(void);
extern int  SDL_QuitInit(void);

/* Event handler quit routines */
extern void SDL_AppActiveQuit(void);
extern void SDL_KeyboardQuit(void);
extern void SDL_MouseQuit(void);
extern void SDL_QuitQuit(void);

/* The event filter function */
extern SDL_EventFilter SDL_EventOK;

/* The array of event processing states */
extern Uint8 SDL_ProcessEvents[SDL_NUMEVENTS];

extern int SDL_PrivateAppActive(Uint8 gain, Uint8 state);
extern int SDL_PrivateMouseMotion(Uint8 buttonstate, int relative,
						Sint16 x, Sint16 y);
extern int SDL_PrivateMouseButton(Uint8 state, Uint8 button,Sint16 x,Sint16 y);
extern int SDL_PrivateKeyboard(Uint8 state, SDL_keysym *key);
extern int SDL_PrivateResize(int w, int h);
extern int SDL_PrivateExpose(void);
extern int SDL_PrivateQuit(void);
extern int SDL_PrivateSysWMEvent(SDL_SysWMmsg *message);

/* Used by the activity event handler to remove mouse focus */
extern void SDL_ResetMouse(void);

/* Used by the activity event handler to remove keyboard focus */
extern void SDL_ResetKeyboard(void);

/* Used by the event loop to queue pending keyboard repeat events */
extern void SDL_CheckKeyRepeat(void);

/* Used by the OS keyboard code to detect whether or not to do UNICODE */
#ifndef DEFAULT_UNICODE_TRANSLATION
#define DEFAULT_UNICODE_TRANSLATION 0	/* Default off because of overhead */
#endif
extern int SDL_TranslateUNICODE;
