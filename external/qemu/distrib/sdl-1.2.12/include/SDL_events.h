

/* Include file for SDL event handling */

#ifndef _SDL_events_h
#define _SDL_events_h

#include "SDL_stdinc.h"
#include "SDL_error.h"
#include "SDL_active.h"
#include "SDL_keyboard.h"
#include "SDL_mouse.h"
#include "SDL_joystick.h"
#include "SDL_quit.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

/* General keyboard/mouse state definitions */
#define SDL_RELEASED	0
#define SDL_PRESSED	1

/* Event enumerations */
typedef enum {
       SDL_NOEVENT = 0,			/* Unused (do not remove) */
       SDL_ACTIVEEVENT,			/* Application loses/gains visibility */
       SDL_KEYDOWN,			/* Keys pressed */
       SDL_KEYUP,			/* Keys released */
       SDL_MOUSEMOTION,			/* Mouse moved */
       SDL_MOUSEBUTTONDOWN,		/* Mouse button pressed */
       SDL_MOUSEBUTTONUP,		/* Mouse button released */
       SDL_JOYAXISMOTION,		/* Joystick axis motion */
       SDL_JOYBALLMOTION,		/* Joystick trackball motion */
       SDL_JOYHATMOTION,		/* Joystick hat position change */
       SDL_JOYBUTTONDOWN,		/* Joystick button pressed */
       SDL_JOYBUTTONUP,			/* Joystick button released */
       SDL_QUIT,			/* User-requested quit */
       SDL_SYSWMEVENT,			/* System specific event */
       SDL_EVENT_RESERVEDA,		/* Reserved for future use.. */
       SDL_EVENT_RESERVEDB,		/* Reserved for future use.. */
       SDL_VIDEORESIZE,			/* User resized video mode */
       SDL_VIDEOEXPOSE,			/* Screen needs to be redrawn */
       SDL_EVENT_RESERVED2,		/* Reserved for future use.. */
       SDL_EVENT_RESERVED3,		/* Reserved for future use.. */
       SDL_EVENT_RESERVED4,		/* Reserved for future use.. */
       SDL_EVENT_RESERVED5,		/* Reserved for future use.. */
       SDL_EVENT_RESERVED6,		/* Reserved for future use.. */
       SDL_EVENT_RESERVED7,		/* Reserved for future use.. */
       /* Events SDL_USEREVENT through SDL_MAXEVENTS-1 are for your use */
       SDL_USEREVENT = 24,
       /* This last event is only for bounding internal arrays
	  It is the number of bits in the event mask datatype -- Uint32
        */
       SDL_NUMEVENTS = 32
} SDL_EventType;

/* Predefined event masks */
#define SDL_EVENTMASK(X)	(1<<(X))
typedef enum {
	SDL_ACTIVEEVENTMASK	= SDL_EVENTMASK(SDL_ACTIVEEVENT),
	SDL_KEYDOWNMASK		= SDL_EVENTMASK(SDL_KEYDOWN),
	SDL_KEYUPMASK		= SDL_EVENTMASK(SDL_KEYUP),
	SDL_KEYEVENTMASK	= SDL_EVENTMASK(SDL_KEYDOWN)|
	                          SDL_EVENTMASK(SDL_KEYUP),
	SDL_MOUSEMOTIONMASK	= SDL_EVENTMASK(SDL_MOUSEMOTION),
	SDL_MOUSEBUTTONDOWNMASK	= SDL_EVENTMASK(SDL_MOUSEBUTTONDOWN),
	SDL_MOUSEBUTTONUPMASK	= SDL_EVENTMASK(SDL_MOUSEBUTTONUP),
	SDL_MOUSEEVENTMASK	= SDL_EVENTMASK(SDL_MOUSEMOTION)|
	                          SDL_EVENTMASK(SDL_MOUSEBUTTONDOWN)|
	                          SDL_EVENTMASK(SDL_MOUSEBUTTONUP),
	SDL_JOYAXISMOTIONMASK	= SDL_EVENTMASK(SDL_JOYAXISMOTION),
	SDL_JOYBALLMOTIONMASK	= SDL_EVENTMASK(SDL_JOYBALLMOTION),
	SDL_JOYHATMOTIONMASK	= SDL_EVENTMASK(SDL_JOYHATMOTION),
	SDL_JOYBUTTONDOWNMASK	= SDL_EVENTMASK(SDL_JOYBUTTONDOWN),
	SDL_JOYBUTTONUPMASK	= SDL_EVENTMASK(SDL_JOYBUTTONUP),
	SDL_JOYEVENTMASK	= SDL_EVENTMASK(SDL_JOYAXISMOTION)|
	                          SDL_EVENTMASK(SDL_JOYBALLMOTION)|
	                          SDL_EVENTMASK(SDL_JOYHATMOTION)|
	                          SDL_EVENTMASK(SDL_JOYBUTTONDOWN)|
	                          SDL_EVENTMASK(SDL_JOYBUTTONUP),
	SDL_VIDEORESIZEMASK	= SDL_EVENTMASK(SDL_VIDEORESIZE),
	SDL_VIDEOEXPOSEMASK	= SDL_EVENTMASK(SDL_VIDEOEXPOSE),
	SDL_QUITMASK		= SDL_EVENTMASK(SDL_QUIT),
	SDL_SYSWMEVENTMASK	= SDL_EVENTMASK(SDL_SYSWMEVENT)
} SDL_EventMask ;
#define SDL_ALLEVENTS		0xFFFFFFFF

/* Application visibility event structure */
typedef struct SDL_ActiveEvent {
	Uint8 type;	/* SDL_ACTIVEEVENT */
	Uint8 gain;	/* Whether given states were gained or lost (1/0) */
	Uint8 state;	/* A mask of the focus states */
} SDL_ActiveEvent;

/* Keyboard event structure */
typedef struct SDL_KeyboardEvent {
	Uint8 type;	/* SDL_KEYDOWN or SDL_KEYUP */
	Uint8 which;	/* The keyboard device index */
	Uint8 state;	/* SDL_PRESSED or SDL_RELEASED */
	SDL_keysym keysym;
} SDL_KeyboardEvent;

/* Mouse motion event structure */
typedef struct SDL_MouseMotionEvent {
	Uint8 type;	/* SDL_MOUSEMOTION */
	Uint8 which;	/* The mouse device index */
	Uint8 state;	/* The current button state */
	Uint16 x, y;	/* The X/Y coordinates of the mouse */
	Sint16 xrel;	/* The relative motion in the X direction */
	Sint16 yrel;	/* The relative motion in the Y direction */
} SDL_MouseMotionEvent;

/* Mouse button event structure */
typedef struct SDL_MouseButtonEvent {
	Uint8 type;	/* SDL_MOUSEBUTTONDOWN or SDL_MOUSEBUTTONUP */
	Uint8 which;	/* The mouse device index */
	Uint8 button;	/* The mouse button index */
	Uint8 state;	/* SDL_PRESSED or SDL_RELEASED */
	Uint16 x, y;	/* The X/Y coordinates of the mouse at press time */
} SDL_MouseButtonEvent;

/* Joystick axis motion event structure */
typedef struct SDL_JoyAxisEvent {
	Uint8 type;	/* SDL_JOYAXISMOTION */
	Uint8 which;	/* The joystick device index */
	Uint8 axis;	/* The joystick axis index */
	Sint16 value;	/* The axis value (range: -32768 to 32767) */
} SDL_JoyAxisEvent;

/* Joystick trackball motion event structure */
typedef struct SDL_JoyBallEvent {
	Uint8 type;	/* SDL_JOYBALLMOTION */
	Uint8 which;	/* The joystick device index */
	Uint8 ball;	/* The joystick trackball index */
	Sint16 xrel;	/* The relative motion in the X direction */
	Sint16 yrel;	/* The relative motion in the Y direction */
} SDL_JoyBallEvent;

/* Joystick hat position change event structure */
typedef struct SDL_JoyHatEvent {
	Uint8 type;	/* SDL_JOYHATMOTION */
	Uint8 which;	/* The joystick device index */
	Uint8 hat;	/* The joystick hat index */
	Uint8 value;	/* The hat position value:
			    SDL_HAT_LEFTUP   SDL_HAT_UP       SDL_HAT_RIGHTUP
			    SDL_HAT_LEFT     SDL_HAT_CENTERED SDL_HAT_RIGHT
			    SDL_HAT_LEFTDOWN SDL_HAT_DOWN     SDL_HAT_RIGHTDOWN
			   Note that zero means the POV is centered.
			*/
} SDL_JoyHatEvent;

/* Joystick button event structure */
typedef struct SDL_JoyButtonEvent {
	Uint8 type;	/* SDL_JOYBUTTONDOWN or SDL_JOYBUTTONUP */
	Uint8 which;	/* The joystick device index */
	Uint8 button;	/* The joystick button index */
	Uint8 state;	/* SDL_PRESSED or SDL_RELEASED */
} SDL_JoyButtonEvent;

typedef struct SDL_ResizeEvent {
	Uint8 type;	/* SDL_VIDEORESIZE */
	int w;		/* New width */
	int h;		/* New height */
} SDL_ResizeEvent;

/* The "screen redraw" event */
typedef struct SDL_ExposeEvent {
	Uint8 type;	/* SDL_VIDEOEXPOSE */
} SDL_ExposeEvent;

/* The "quit requested" event */
typedef struct SDL_QuitEvent {
	Uint8 type;	/* SDL_QUIT */
} SDL_QuitEvent;

/* A user-defined event type */
typedef struct SDL_UserEvent {
	Uint8 type;	/* SDL_USEREVENT through SDL_NUMEVENTS-1 */
	int code;	/* User defined event code */
	void *data1;	/* User defined data pointer */
	void *data2;	/* User defined data pointer */
} SDL_UserEvent;

/* If you want to use this event, you should include SDL_syswm.h */
struct SDL_SysWMmsg;
typedef struct SDL_SysWMmsg SDL_SysWMmsg;
typedef struct SDL_SysWMEvent {
	Uint8 type;
	SDL_SysWMmsg *msg;
} SDL_SysWMEvent;

/* General event structure */
typedef union SDL_Event {
	Uint8 type;
	SDL_ActiveEvent active;
	SDL_KeyboardEvent key;
	SDL_MouseMotionEvent motion;
	SDL_MouseButtonEvent button;
	SDL_JoyAxisEvent jaxis;
	SDL_JoyBallEvent jball;
	SDL_JoyHatEvent jhat;
	SDL_JoyButtonEvent jbutton;
	SDL_ResizeEvent resize;
	SDL_ExposeEvent expose;
	SDL_QuitEvent quit;
	SDL_UserEvent user;
	SDL_SysWMEvent syswm;
} SDL_Event;


/* Function prototypes */

extern DECLSPEC void SDLCALL SDL_PumpEvents(void);

typedef enum {
	SDL_ADDEVENT,
	SDL_PEEKEVENT,
	SDL_GETEVENT
} SDL_eventaction;
/* */
extern DECLSPEC int SDLCALL SDL_PeepEvents(SDL_Event *events, int numevents,
				SDL_eventaction action, Uint32 mask);

extern DECLSPEC int SDLCALL SDL_PollEvent(SDL_Event *event);

extern DECLSPEC int SDLCALL SDL_WaitEvent(SDL_Event *event);

extern DECLSPEC int SDLCALL SDL_PushEvent(SDL_Event *event);

typedef int (SDLCALL *SDL_EventFilter)(const SDL_Event *event);
extern DECLSPEC void SDLCALL SDL_SetEventFilter(SDL_EventFilter filter);

extern DECLSPEC SDL_EventFilter SDLCALL SDL_GetEventFilter(void);

#define SDL_QUERY	-1
#define SDL_IGNORE	 0
#define SDL_DISABLE	 0
#define SDL_ENABLE	 1
extern DECLSPEC Uint8 SDLCALL SDL_EventState(Uint8 type, int state);


/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_events_h */
