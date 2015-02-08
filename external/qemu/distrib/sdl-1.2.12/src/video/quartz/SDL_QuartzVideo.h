
#include "SDL_config.h"


/* Needs to be first, so QuickTime.h doesn't include glext.h (10.4) */
#include "SDL_opengl.h"

#include <Cocoa/Cocoa.h>
#include <Carbon/Carbon.h>
#include <QuickTime/QuickTime.h>
#include <OpenGL/OpenGL.h>	/* For CGL functions and types */
#include <IOKit/IOKitLib.h>	/* For powersave handling */
#include <pthread.h>

#include "SDL_thread.h"
#include "SDL_video.h"
#include "SDL_error.h"
#include "SDL_timer.h"
#include "SDL_loadso.h"
#include "SDL_syswm.h"
#include "../SDL_sysvideo.h"
#include "../SDL_pixels_c.h"
#include "../../events/SDL_events_c.h"

@interface NSOpenGLContext (CGLContextAccess)
- (CGLContextObj) cglContext;
@end


/* Main driver structure to store required state information */
typedef struct SDL_PrivateVideoData {

    BOOL               allow_screensaver;  /* 0 == disable screensaver */
    CGDirectDisplayID  display;            /* 0 == main display (only support single display) */
    CFDictionaryRef    mode;               /* current mode of the display */
    CFDictionaryRef    save_mode;          /* original mode of the display */
    CFArrayRef         mode_list;          /* list of available fullscreen modes */
    CGDirectPaletteRef palette;            /* palette of an 8-bit display */
    NSOpenGLContext    *gl_context;        /* OpenGL rendering context */
    Uint32             width, height, bpp; /* frequently used data about the display */
    Uint32             flags;              /* flags for current mode, for teardown purposes */
    Uint32             video_set;          /* boolean; indicates if video was set correctly */
    Uint32             warp_flag;          /* boolean; notify to event loop that a warp just occured */
    Uint32             warp_ticks;         /* timestamp when the warp occured */
    NSWindow           *window;            /* Cocoa window to implement the SDL window */
    NSQuickDrawView    *view;              /* the window's view; draw 2D and OpenGL into this view */
    SDL_Surface        *resize_icon;       /* icon for the resize badge, we have to draw it by hand */
    SDL_GrabMode       current_grab_mode;  /* default value is SDL_GRAB_OFF */
    SDL_Rect           **client_mode_list; /* resolution list to pass back to client */
    SDLKey             keymap[256];        /* Mac OS X to SDL key mapping */
    Uint32             current_mods;       /* current keyboard modifiers, to track modifier state */
    NSText             *field_edit;        /* a field editor for keyboard composition processing */
    Uint32             last_virtual_button;/* last virtual mouse button pressed */
    io_connect_t       power_connection;   /* used with IOKit to detect wake from sleep */
    Uint8              expect_mouse_up;    /* used to determine when to send mouse up events */
    Uint8              grab_state;         /* used to manage grab behavior */
    NSPoint            cursor_loc;         /* saved cursor coords, for activate/deactivate when grabbed */
    BOOL               cursor_should_be_visible;     /* tells if cursor is supposed to be visible (SDL_ShowCursor) */
    BOOL               cursor_visible;     /* tells if cursor is *actually* visible or not */
    Uint8*             sw_buffers[2];      /* pointers to the two software buffers for double-buffer emulation */
    SDL_Thread         *thread;            /* thread for async updates to the screen */
    SDL_sem            *sem1, *sem2;       /* synchronization for async screen updates */
    Uint8              *current_buffer;    /* the buffer being copied to the screen */
    BOOL               quit_thread;        /* used to quit the async blitting thread */
    SInt32             system_version;     /* used to dis-/enable workarounds depending on the system version */
    
    ImageDescriptionHandle yuv_idh;
    MatrixRecordPtr        yuv_matrix;
    DecompressorComponent  yuv_codec;
    ImageSequence          yuv_seq;
    PlanarPixmapInfoYUV420 *yuv_pixmap;
    Sint16                  yuv_width, yuv_height;
    CGrafPtr                yuv_port;

    void *opengl_library;    /* dynamically loaded OpenGL library. */
} SDL_PrivateVideoData;

#define _THIS    SDL_VideoDevice *this
#define display_id (this->hidden->display)
#define mode (this->hidden->mode)
#define save_mode (this->hidden->save_mode)
#define allow_screensaver (this->hidden->allow_screensaver)
#define mode_list (this->hidden->mode_list)
#define palette (this->hidden->palette)
#define gl_context (this->hidden->gl_context)
#define device_width (this->hidden->width)
#define device_height (this->hidden->height)
#define device_bpp (this->hidden->bpp)
#define mode_flags (this->hidden->flags)
#define qz_window (this->hidden->window)
#define window_view (this->hidden->view)
#define video_set (this->hidden->video_set)
#define warp_ticks (this->hidden->warp_ticks)
#define warp_flag (this->hidden->warp_flag)
#define resize_icon (this->hidden->resize_icon)
#define current_grab_mode (this->hidden->current_grab_mode)
#define client_mode_list (this->hidden->client_mode_list)
#define keymap (this->hidden->keymap)
#define current_mods (this->hidden->current_mods)
#define field_edit (this->hidden->field_edit)
#define last_virtual_button (this->hidden->last_virtual_button)
#define power_connection (this->hidden->power_connection)
#define expect_mouse_up (this->hidden->expect_mouse_up)
#define grab_state (this->hidden->grab_state)
#define cursor_loc (this->hidden->cursor_loc)
#define cursor_should_be_visible (this->hidden->cursor_should_be_visible)
#define cursor_visible (this->hidden->cursor_visible)
#define sw_buffers (this->hidden->sw_buffers)
#define thread (this->hidden->thread)
#define sem1 (this->hidden->sem1)
#define sem2 (this->hidden->sem2)
#define current_buffer (this->hidden->current_buffer)
#define quit_thread (this->hidden->quit_thread)
#define system_version (this->hidden->system_version)
#define opengl_library (this->hidden->opengl_library)

/* grab states - the input is in one of these states */
enum {
    QZ_UNGRABBED = 0,
    QZ_VISIBLE_GRAB,
    QZ_INVISIBLE_GRAB
};

/* grab actions - these can change the grabbed state */
enum {
    QZ_ENABLE_GRAB = 0,
    QZ_DISABLE_GRAB,
    QZ_HIDECURSOR,
    QZ_SHOWCURSOR
};

/* Gamma Functions */
int    QZ_SetGamma          (_THIS, float red, float green, float blue);
int    QZ_GetGamma          (_THIS, float *red, float *green, float *blue);
int    QZ_SetGammaRamp      (_THIS, Uint16 *ramp);
int    QZ_GetGammaRamp      (_THIS, Uint16 *ramp);

/* OpenGL functions */
int    QZ_SetupOpenGL       (_THIS, int bpp, Uint32 flags);
void   QZ_TearDownOpenGL    (_THIS);
void*  QZ_GL_GetProcAddress (_THIS, const char *proc);
int    QZ_GL_GetAttribute   (_THIS, SDL_GLattr attrib, int* value);
int    QZ_GL_MakeCurrent    (_THIS);
void   QZ_GL_SwapBuffers    (_THIS);
int    QZ_GL_LoadLibrary    (_THIS, const char *location);

/* Cursor and Mouse functions */
void         QZ_FreeWMCursor     (_THIS, WMcursor *cursor);
WMcursor*    QZ_CreateWMCursor   (_THIS, Uint8 *data, Uint8 *mask,
                                  int w, int h, int hot_x, int hot_y);
int          QZ_ShowWMCursor     (_THIS, WMcursor *cursor);
void         QZ_WarpWMCursor     (_THIS, Uint16 x, Uint16 y);
void         QZ_MoveWMCursor     (_THIS, int x, int y);
void         QZ_CheckMouseMode   (_THIS);
void         QZ_UpdateMouse      (_THIS);

/* Event functions */
void         QZ_InitOSKeymap     (_THIS);
void         QZ_PumpEvents       (_THIS);

/* Window Manager functions */
void         QZ_SetCaption       (_THIS, const char *title, const char *icon);
void         QZ_SetIcon          (_THIS, SDL_Surface *icon, Uint8 *mask);
int          QZ_IconifyWindow    (_THIS);
void         QZ_SetWindowPos     (_THIS, int  x, int  y);
void         QZ_GetWindowPos     (_THIS, int  *px, int  *py);
int          QZ_IsWindowVisible  (_THIS, int  recenter);
int          QZ_GetMonitorDPI    (_THIS, int  *xDpi, int *yDpi);
int          QZ_GetMonitorRect   (_THIS, SDL_Rect  *rect);

SDL_GrabMode QZ_GrabInput        (_THIS, SDL_GrabMode grab_mode);
int          QZ_GetWMInfo        (_THIS, SDL_SysWMinfo *info);

/* YUV functions */
SDL_Overlay* QZ_CreateYUVOverlay (_THIS, int width, int height,
                                         Uint32 format, SDL_Surface *display);


/* Private functions (used internally) */
void         QZ_PrivateWarpCursor (_THIS, int x, int y);
void         QZ_ChangeGrabState (_THIS, int action);
void         QZ_RegisterForSleepNotifications (_THIS);
void         QZ_PrivateGlobalToLocal (_THIS, NSPoint *p);
void         QZ_PrivateCocoaToSDL (_THIS, NSPoint *p);
BOOL         QZ_IsMouseInWindow (_THIS);
void         QZ_DoActivate (_THIS);
void         QZ_DoDeactivate (_THIS);
