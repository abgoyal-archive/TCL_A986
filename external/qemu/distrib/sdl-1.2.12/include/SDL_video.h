

/* Header file for access to the SDL raw framebuffer window */

#ifndef _SDL_video_h
#define _SDL_video_h

#include "SDL_stdinc.h"
#include "SDL_error.h"
#include "SDL_rwops.h"

#include "begin_code.h"
/* Set up for C function definitions, even when using C++ */
#ifdef __cplusplus
extern "C" {
#endif

/* Transparency definitions: These define alpha as the opacity of a surface */
#define SDL_ALPHA_OPAQUE 255
#define SDL_ALPHA_TRANSPARENT 0

/* Useful data types */
typedef struct SDL_Rect {
	Sint16 x, y;
	Uint16 w, h;
} SDL_Rect;

typedef struct SDL_Color {
	Uint8 r;
	Uint8 g;
	Uint8 b;
	Uint8 unused;
} SDL_Color;
#define SDL_Colour SDL_Color

typedef struct SDL_Palette {
	int       ncolors;
	SDL_Color *colors;
} SDL_Palette;

/* Everything in the pixel format structure is read-only */
typedef struct SDL_PixelFormat {
	SDL_Palette *palette;
	Uint8  BitsPerPixel;
	Uint8  BytesPerPixel;
	Uint8  Rloss;
	Uint8  Gloss;
	Uint8  Bloss;
	Uint8  Aloss;
	Uint8  Rshift;
	Uint8  Gshift;
	Uint8  Bshift;
	Uint8  Ashift;
	Uint32 Rmask;
	Uint32 Gmask;
	Uint32 Bmask;
	Uint32 Amask;

	/* RGB color key information */
	Uint32 colorkey;
	/* Alpha value information (per-surface alpha) */
	Uint8  alpha;
} SDL_PixelFormat;

typedef struct SDL_Surface {
	Uint32 flags;				/* Read-only */
	SDL_PixelFormat *format;		/* Read-only */
	int w, h;				/* Read-only */
	Uint16 pitch;				/* Read-only */
	void *pixels;				/* Read-write */
	int offset;				/* Private */

	/* Hardware-specific surface info */
	struct private_hwdata *hwdata;

	/* clipping information */
	SDL_Rect clip_rect;			/* Read-only */
	Uint32 unused1;				/* for binary compatibility */

	/* Allow recursive locks */
	Uint32 locked;				/* Private */

	/* info for fast blit mapping to other surfaces */
	struct SDL_BlitMap *map;		/* Private */

	/* format version, bumped at every change to invalidate blit maps */
	unsigned int format_version;		/* Private */

	/* Reference count -- used when freeing surface */
	int refcount;				/* Read-mostly */
} SDL_Surface;

/* These are the currently supported flags for the SDL_surface */
/* Available for SDL_CreateRGBSurface() or SDL_SetVideoMode() */
#define SDL_SWSURFACE	0x00000000	/* Surface is in system memory */
#define SDL_HWSURFACE	0x00000001	/* Surface is in video memory */
#define SDL_ASYNCBLIT	0x00000004	/* Use asynchronous blits if possible */
/* Available for SDL_SetVideoMode() */
#define SDL_ANYFORMAT	0x10000000	/* Allow any video depth/pixel-format */
#define SDL_HWPALETTE	0x20000000	/* Surface has exclusive palette */
#define SDL_DOUBLEBUF	0x40000000	/* Set up double-buffered video mode */
#define SDL_FULLSCREEN	0x80000000	/* Surface is a full screen display */
#define SDL_OPENGL      0x00000002      /* Create an OpenGL rendering context */
#define SDL_OPENGLBLIT	0x0000000A	/* Create an OpenGL rendering context and use it for blitting */
#define SDL_RESIZABLE	0x00000010	/* This video mode may be resized */
#define SDL_NOFRAME	0x00000020	/* No window caption or edge frame */
/* Used internally (read-only) */
#define SDL_HWACCEL	0x00000100	/* Blit uses hardware acceleration */
#define SDL_SRCCOLORKEY	0x00001000	/* Blit uses a source color key */
#define SDL_RLEACCELOK	0x00002000	/* Private flag */
#define SDL_RLEACCEL	0x00004000	/* Surface is RLE encoded */
#define SDL_SRCALPHA	0x00010000	/* Blit uses source alpha blending */
#define SDL_PREALLOC	0x01000000	/* Surface uses preallocated memory */

/* Evaluates to true if the surface needs to be locked before access */
#define SDL_MUSTLOCK(surface)	\
  (surface->offset ||		\
  ((surface->flags & (SDL_HWSURFACE|SDL_ASYNCBLIT|SDL_RLEACCEL)) != 0))

/* typedef for private surface blitting functions */
typedef int (*SDL_blit)(struct SDL_Surface *src, SDL_Rect *srcrect,
			struct SDL_Surface *dst, SDL_Rect *dstrect);


/* Useful for determining the video hardware capabilities */
typedef struct SDL_VideoInfo {
	Uint32 hw_available :1;	/* Flag: Can you create hardware surfaces? */
	Uint32 wm_available :1;	/* Flag: Can you talk to a window manager? */
	Uint32 UnusedBits1  :6;
	Uint32 UnusedBits2  :1;
	Uint32 blit_hw      :1;	/* Flag: Accelerated blits HW --> HW */
	Uint32 blit_hw_CC   :1;	/* Flag: Accelerated blits with Colorkey */
	Uint32 blit_hw_A    :1;	/* Flag: Accelerated blits with Alpha */
	Uint32 blit_sw      :1;	/* Flag: Accelerated blits SW --> HW */
	Uint32 blit_sw_CC   :1;	/* Flag: Accelerated blits with Colorkey */
	Uint32 blit_sw_A    :1;	/* Flag: Accelerated blits with Alpha */
	Uint32 blit_fill    :1;	/* Flag: Accelerated color fill */
	Uint32 UnusedBits3  :16;
	Uint32 video_mem;	/* The total amount of video memory (in K) */
	SDL_PixelFormat *vfmt;	/* Value: The format of the video surface */
	int    current_w;	/* Value: The current video mode width */
	int    current_h;	/* Value: The current video mode height */
} SDL_VideoInfo;


#define SDL_YV12_OVERLAY  0x32315659	/* Planar mode: Y + V + U  (3 planes) */
#define SDL_IYUV_OVERLAY  0x56555949	/* Planar mode: Y + U + V  (3 planes) */
#define SDL_YUY2_OVERLAY  0x32595559	/* Packed mode: Y0+U0+Y1+V0 (1 plane) */
#define SDL_UYVY_OVERLAY  0x59565955	/* Packed mode: U0+Y0+V0+Y1 (1 plane) */
#define SDL_YVYU_OVERLAY  0x55595659	/* Packed mode: Y0+V0+Y1+U0 (1 plane) */

/* The YUV hardware video overlay */
typedef struct SDL_Overlay {
	Uint32 format;				/* Read-only */
	int w, h;				/* Read-only */
	int planes;				/* Read-only */
	Uint16 *pitches;			/* Read-only */
	Uint8 **pixels;				/* Read-write */

	/* Hardware-specific surface info */
	struct private_yuvhwfuncs *hwfuncs;
	struct private_yuvhwdata *hwdata;

	/* Special flags */
	Uint32 hw_overlay :1;	/* Flag: This overlay hardware accelerated? */
	Uint32 UnusedBits :31;
} SDL_Overlay;


/* Public enumeration for setting the OpenGL window attributes. */
typedef enum {
    SDL_GL_RED_SIZE,
    SDL_GL_GREEN_SIZE,
    SDL_GL_BLUE_SIZE,
    SDL_GL_ALPHA_SIZE,
    SDL_GL_BUFFER_SIZE,
    SDL_GL_DOUBLEBUFFER,
    SDL_GL_DEPTH_SIZE,
    SDL_GL_STENCIL_SIZE,
    SDL_GL_ACCUM_RED_SIZE,
    SDL_GL_ACCUM_GREEN_SIZE,
    SDL_GL_ACCUM_BLUE_SIZE,
    SDL_GL_ACCUM_ALPHA_SIZE,
    SDL_GL_STEREO,
    SDL_GL_MULTISAMPLEBUFFERS,
    SDL_GL_MULTISAMPLESAMPLES,
    SDL_GL_ACCELERATED_VISUAL,
    SDL_GL_SWAP_CONTROL
} SDL_GLattr;

/* flags for SDL_SetPalette() */
#define SDL_LOGPAL 0x01
#define SDL_PHYSPAL 0x02

/* Function prototypes */

extern DECLSPEC int SDLCALL SDL_VideoInit(const char *driver_name, Uint32 flags);
extern DECLSPEC void SDLCALL SDL_VideoQuit(void);

extern DECLSPEC char * SDLCALL SDL_VideoDriverName(char *namebuf, int maxlen);

extern DECLSPEC SDL_Surface * SDLCALL SDL_GetVideoSurface(void);

extern DECLSPEC const SDL_VideoInfo * SDLCALL SDL_GetVideoInfo(void);

extern DECLSPEC int SDLCALL SDL_VideoModeOK(int width, int height, int bpp, Uint32 flags);

extern DECLSPEC SDL_Rect ** SDLCALL SDL_ListModes(SDL_PixelFormat *format, Uint32 flags);

extern DECLSPEC SDL_Surface * SDLCALL SDL_SetVideoMode
			(int width, int height, int bpp, Uint32 flags);

extern DECLSPEC void SDLCALL SDL_UpdateRects
		(SDL_Surface *screen, int numrects, SDL_Rect *rects);
extern DECLSPEC void SDLCALL SDL_UpdateRect
		(SDL_Surface *screen, Sint32 x, Sint32 y, Uint32 w, Uint32 h);

extern DECLSPEC int SDLCALL SDL_Flip(SDL_Surface *screen);

extern DECLSPEC int SDLCALL SDL_SetGamma(float red, float green, float blue);

extern DECLSPEC int SDLCALL SDL_SetGammaRamp(const Uint16 *red, const Uint16 *green, const Uint16 *blue);

extern DECLSPEC int SDLCALL SDL_GetGammaRamp(Uint16 *red, Uint16 *green, Uint16 *blue);

extern DECLSPEC int SDLCALL SDL_SetColors(SDL_Surface *surface, 
			SDL_Color *colors, int firstcolor, int ncolors);

extern DECLSPEC int SDLCALL SDL_SetPalette(SDL_Surface *surface, int flags,
				   SDL_Color *colors, int firstcolor,
				   int ncolors);

extern DECLSPEC Uint32 SDLCALL SDL_MapRGB
(const SDL_PixelFormat * const format,
 const Uint8 r, const Uint8 g, const Uint8 b);

extern DECLSPEC Uint32 SDLCALL SDL_MapRGBA
(const SDL_PixelFormat * const format,
 const Uint8 r, const Uint8 g, const Uint8 b, const Uint8 a);

extern DECLSPEC void SDLCALL SDL_GetRGB(Uint32 pixel, SDL_PixelFormat *fmt,
				Uint8 *r, Uint8 *g, Uint8 *b);

extern DECLSPEC void SDLCALL SDL_GetRGBA(Uint32 pixel, SDL_PixelFormat *fmt,
				 Uint8 *r, Uint8 *g, Uint8 *b, Uint8 *a);

#define SDL_AllocSurface    SDL_CreateRGBSurface
extern DECLSPEC SDL_Surface * SDLCALL SDL_CreateRGBSurface
			(Uint32 flags, int width, int height, int depth, 
			Uint32 Rmask, Uint32 Gmask, Uint32 Bmask, Uint32 Amask);
extern DECLSPEC SDL_Surface * SDLCALL SDL_CreateRGBSurfaceFrom(void *pixels,
			int width, int height, int depth, int pitch,
			Uint32 Rmask, Uint32 Gmask, Uint32 Bmask, Uint32 Amask);
extern DECLSPEC void SDLCALL SDL_FreeSurface(SDL_Surface *surface);

extern DECLSPEC int SDLCALL SDL_LockSurface(SDL_Surface *surface);
extern DECLSPEC void SDLCALL SDL_UnlockSurface(SDL_Surface *surface);

extern DECLSPEC SDL_Surface * SDLCALL SDL_LoadBMP_RW(SDL_RWops *src, int freesrc);

/* Convenience macro -- load a surface from a file */
#define SDL_LoadBMP(file)	SDL_LoadBMP_RW(SDL_RWFromFile(file, "rb"), 1)

extern DECLSPEC int SDLCALL SDL_SaveBMP_RW
		(SDL_Surface *surface, SDL_RWops *dst, int freedst);

/* Convenience macro -- save a surface to a file */
#define SDL_SaveBMP(surface, file) \
		SDL_SaveBMP_RW(surface, SDL_RWFromFile(file, "wb"), 1)

extern DECLSPEC int SDLCALL SDL_SetColorKey
			(SDL_Surface *surface, Uint32 flag, Uint32 key);

extern DECLSPEC int SDLCALL SDL_SetAlpha(SDL_Surface *surface, Uint32 flag, Uint8 alpha);

extern DECLSPEC SDL_bool SDLCALL SDL_SetClipRect(SDL_Surface *surface, const SDL_Rect *rect);

extern DECLSPEC void SDLCALL SDL_GetClipRect(SDL_Surface *surface, SDL_Rect *rect);

extern DECLSPEC SDL_Surface * SDLCALL SDL_ConvertSurface
			(SDL_Surface *src, SDL_PixelFormat *fmt, Uint32 flags);

#define SDL_BlitSurface SDL_UpperBlit

extern DECLSPEC int SDLCALL SDL_UpperBlit
			(SDL_Surface *src, SDL_Rect *srcrect,
			 SDL_Surface *dst, SDL_Rect *dstrect);
extern DECLSPEC int SDLCALL SDL_LowerBlit
			(SDL_Surface *src, SDL_Rect *srcrect,
			 SDL_Surface *dst, SDL_Rect *dstrect);

extern DECLSPEC int SDLCALL SDL_FillRect
		(SDL_Surface *dst, SDL_Rect *dstrect, Uint32 color);

extern DECLSPEC SDL_Surface * SDLCALL SDL_DisplayFormat(SDL_Surface *surface);

extern DECLSPEC SDL_Surface * SDLCALL SDL_DisplayFormatAlpha(SDL_Surface *surface);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* YUV video surface overlay functions                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

extern DECLSPEC SDL_Overlay * SDLCALL SDL_CreateYUVOverlay(int width, int height,
				Uint32 format, SDL_Surface *display);

/* Lock an overlay for direct access, and unlock it when you are done */
extern DECLSPEC int SDLCALL SDL_LockYUVOverlay(SDL_Overlay *overlay);
extern DECLSPEC void SDLCALL SDL_UnlockYUVOverlay(SDL_Overlay *overlay);

extern DECLSPEC int SDLCALL SDL_DisplayYUVOverlay(SDL_Overlay *overlay, SDL_Rect *dstrect);

/* Free a video overlay */
extern DECLSPEC void SDLCALL SDL_FreeYUVOverlay(SDL_Overlay *overlay);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* OpenGL support functions.                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

extern DECLSPEC int SDLCALL SDL_GL_LoadLibrary(const char *path);

extern DECLSPEC void * SDLCALL SDL_GL_GetProcAddress(const char* proc);

extern DECLSPEC int SDLCALL SDL_GL_SetAttribute(SDL_GLattr attr, int value);

extern DECLSPEC int SDLCALL SDL_GL_GetAttribute(SDL_GLattr attr, int* value);

extern DECLSPEC void SDLCALL SDL_GL_SwapBuffers(void);

extern DECLSPEC void SDLCALL SDL_GL_UpdateRects(int numrects, SDL_Rect* rects);
extern DECLSPEC void SDLCALL SDL_GL_Lock(void);
extern DECLSPEC void SDLCALL SDL_GL_Unlock(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* These functions allow interaction with the window manager, if any.        */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

extern DECLSPEC void SDLCALL SDL_WM_SetCaption(const char *title, const char *icon);
extern DECLSPEC void SDLCALL SDL_WM_GetCaption(char **title, char **icon);

extern DECLSPEC void SDLCALL SDL_WM_SetIcon(SDL_Surface *icon, Uint8 *mask);

extern DECLSPEC int SDLCALL SDL_WM_IconifyWindow(void);

extern DECLSPEC int SDLCALL SDL_WM_ToggleFullScreen(SDL_Surface *surface);

typedef enum {
	SDL_GRAB_QUERY = -1,
	SDL_GRAB_OFF = 0,
	SDL_GRAB_ON = 1,
	SDL_GRAB_FULLSCREEN	/* Used internally */
} SDL_GrabMode;
extern DECLSPEC SDL_GrabMode SDLCALL SDL_WM_GrabInput(SDL_GrabMode mode);

extern DECLSPEC void  SDL_WM_SetPos(int  x,  int  y);

extern DECLSPEC void  SDL_WM_GetPos(int  *px, int  *py);

extern DECLSPEC int   SDL_WM_IsFullyVisible( int  recenter );

extern DECLSPEC int   SDL_WM_GetMonitorDPI( int  *xDpi, int  *yDpi );

extern DECLSPEC int   SDL_WM_GetMonitorRect( SDL_Rect  *rect );

/* Not in public API at the moment - do not use! */
extern DECLSPEC int SDLCALL SDL_SoftStretch(SDL_Surface *src, SDL_Rect *srcrect,
                                    SDL_Surface *dst, SDL_Rect *dstrect);
                    
/* Ends C function definitions when using C++ */
#ifdef __cplusplus
}
#endif
#include "close_code.h"

#endif /* _SDL_video_h */
