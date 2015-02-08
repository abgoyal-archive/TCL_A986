
/* $XFree86: xc/include/extensions/Xvlib.h,v 1.3 1999/12/11 19:28:48 mvojkovi Exp $ */

#ifndef XVLIB_H
#define XVLIB_H

#include <X11/Xfuncproto.h>
#include "Xv.h"
#include "SDL_name.h"

typedef struct {
  int numerator;
  int denominator;
} SDL_NAME(XvRational);

typedef struct {
  int flags;	/* XvGettable, XvSettable */
  int min_value;
  int max_value;
  char *name;
} SDL_NAME(XvAttribute);

typedef struct {
  XvEncodingID encoding_id;
  char *name;
  unsigned long width;
  unsigned long height;
  SDL_NAME(XvRational) rate;
  unsigned long num_encodings;
} SDL_NAME(XvEncodingInfo);

typedef struct {
  char depth;
  unsigned long visual_id;
} SDL_NAME(XvFormat);

typedef struct {
  XvPortID base_id;
  unsigned long num_ports;
  char type;
  char *name;
  unsigned long num_formats;
  SDL_NAME(XvFormat) *formats;
  unsigned long num_adaptors;
} SDL_NAME(XvAdaptorInfo);

typedef struct {
  int type;
  unsigned long serial;	   /* # of last request processed by server */
  Bool send_event;	   /* true if this came from a SendEvent request */
  Display *display;	   /* Display the event was read from */
  Drawable drawable;       /* drawable */
  unsigned long reason;    /* what generated this event */
  XvPortID port_id;        /* what port */
  Time time;		   /* milliseconds */
} SDL_NAME(XvVideoNotifyEvent);

typedef struct {
  int type;
  unsigned long serial;	   /* # of last request processed by server */
  Bool send_event;	   /* true if this came from a SendEvent request */
  Display *display;	   /* Display the event was read from */
  XvPortID port_id;        /* what port */
  Time time;		   /* milliseconds */
  Atom attribute;           /* atom that identifies attribute */
  long value;              /* value of attribute */
} SDL_NAME(XvPortNotifyEvent);

typedef union {
  int type;
  SDL_NAME(XvVideoNotifyEvent) xvvideo;
  SDL_NAME(XvPortNotifyEvent) xvport;
  long pad[24];
} SDL_NAME(XvEvent);

typedef struct {
  int id;                      /* Unique descriptor for the format */
  int type;                    /* XvRGB, XvYUV */
  int byte_order;              /* LSBFirst, MSBFirst */
  char guid[16];               /* Globally Unique IDentifier */
  int bits_per_pixel;
  int format;                  /* XvPacked, XvPlanar */
  int num_planes;

  /* for RGB formats only */
  int depth;
  unsigned int red_mask;       
  unsigned int green_mask;   
  unsigned int blue_mask;   

  /* for YUV formats only */
  unsigned int y_sample_bits;
  unsigned int u_sample_bits;
  unsigned int v_sample_bits;   
  unsigned int horz_y_period;
  unsigned int horz_u_period;
  unsigned int horz_v_period;
  unsigned int vert_y_period;
  unsigned int vert_u_period;
  unsigned int vert_v_period;
  char component_order[32];    /* eg. UYVY */
  int scanline_order;          /* XvTopToBottom, XvBottomToTop */
} SDL_NAME(XvImageFormatValues); 

typedef struct {
  int id;
  int width, height;
  int data_size;              /* bytes */
  int num_planes;
  int *pitches;               /* bytes */
  int *offsets;               /* bytes */
  char *data;
  XPointer obdata;     
} SDL_NAME(XvImage);

_XFUNCPROTOBEGIN

extern int SDL_NAME(XvQueryExtension)(
#if NeedFunctionPrototypes
  Display*                 /* display */,
  unsigned int*            /* p_version */,
  unsigned int*            /* p_revision */,
  unsigned int*            /* p_requestBase */,
  unsigned int*            /* p_eventBase */, 
  unsigned int*            /* p_errorBase */
#endif
);

extern int SDL_NAME(XvQueryAdaptors)(
#if NeedFunctionPrototypes
  Display*                 /* display */,
  Window                   /* window */,
  unsigned int*            /* p_nAdaptors */,
  SDL_NAME(XvAdaptorInfo)**          /* p_pAdaptors */
#endif
);

extern int SDL_NAME(XvQueryEncodings)(
#if NeedFunctionPrototypes
  Display*                 /* display */,
  XvPortID                 /* port */,
  unsigned int*            /* p_nEncoding */,
  SDL_NAME(XvEncodingInfo)**         /* p_pEncoding */
#endif
);

extern int SDL_NAME(XvPutVideo)(
#if NeedFunctionPrototypes
  Display*                 /* display */,
  XvPortID                 /* port */,
  Drawable                 /* d */,
  GC                       /* gc */,
  int                      /* vx */, 
  int                      /* vy */,
  unsigned int             /* vw */, 
  unsigned int             /* vh */,
  int                      /* dx */, 
  int                      /* dy */,
  unsigned int             /* dw */,
  unsigned int             /* dh */
#endif
);

extern int SDL_NAME(XvPutStill)(
#if NeedFunctionPrototypes
  Display*                 /* display */,
  XvPortID                 /* port */,
  Drawable                 /* d */,
  GC                       /* gc */,
  int                      /* vx */, 
  int                      /* vy */,
  unsigned int             /* vw */, 
  unsigned int             /* vh */,
  int                      /* dx */, 
  int                      /* dy */,
  unsigned int             /* dw */,
  unsigned int             /* dh */
#endif
);

extern int SDL_NAME(XvGetVideo)(
#if NeedFunctionPrototypes
  Display*                 /* display */,
  XvPortID                 /* port */,
  Drawable                 /* d */,
  GC                       /* gc */,
  int                      /* vx */, 
  int                      /* vy */,
  unsigned int             /* vw */, 
  unsigned int             /* vh */,
  int                      /* dx */, 
  int                      /* dy */,
  unsigned int             /* dw */,
  unsigned int             /* dh */
#endif
);

extern int SDL_NAME(XvGetStill)(
#if NeedFunctionPrototypes
  Display*                 /* display */,
  XvPortID                 /* port */,
  Drawable                 /* d */,
  GC                       /* gc */,
  int                      /* vx */, 
  int                      /* vy */,
  unsigned int             /* vw */, 
  unsigned int             /* vh */,
  int                      /* dx */, 
  int                      /* dy */,
  unsigned int             /* dw */,
  unsigned int             /* dh */
#endif
);

extern int SDL_NAME(XvStopVideo)(
#if NeedFunctionPrototypes
  Display*                /* display */,
  XvPortID                /* port */,
  Drawable                /* drawable */
#endif
);

extern int SDL_NAME(XvGrabPort)(
#if NeedFunctionPrototypes
  Display*                /* display */,
  XvPortID                /* port */,
  Time                    /* time */
#endif
);

extern int SDL_NAME(XvUngrabPort)(
#if NeedFunctionPrototypes
  Display*                /* display */,
  XvPortID                /* port */,
  Time                    /* time */
#endif
);

extern int SDL_NAME(XvSelectVideoNotify)(
#if NeedFunctionPrototypes
  Display*                /* display */,
  Drawable                /* drawable */,
  Bool                    /* onoff */
#endif
);

extern int SDL_NAME(XvSelectPortNotify)(
#if NeedFunctionPrototypes
  Display*                /* display */,
  XvPortID                /* port */,
  Bool                    /* onoff */
#endif
);

extern int SDL_NAME(XvSetPortAttribute)(
#if NeedFunctionPrototypes
  Display*                /* display */,
  XvPortID                /* port */,
  Atom                    /* attribute */,
  int                     /* value */
#endif
);

extern int SDL_NAME(XvGetPortAttribute)(
#if NeedFunctionPrototypes
  Display*                /* display */,
  XvPortID                /* port */,
  Atom                    /* attribute */,
  int*                    /* p_value */
#endif
);

extern int SDL_NAME(XvQueryBestSize)(
#if NeedFunctionPrototypes
  Display*                /* display */,
  XvPortID                /* port */,
  Bool                    /* motion */,
  unsigned int            /* vid_w */, 
  unsigned int            /* vid_h */,
  unsigned int            /* drw_w */, 
  unsigned int            /* drw_h */,
  unsigned int*           /* p_actual_width */, 
  unsigned int*           /* p_actual_width */
#endif
);

extern SDL_NAME(XvAttribute)* SDL_NAME(XvQueryPortAttributes)(
#if NeedFunctionPrototypes
  Display*                /* display */,
  XvPortID                /* port */,
  int*                    /* number */
#endif
);


extern void SDL_NAME(XvFreeAdaptorInfo)(
#if NeedFunctionPrototypes
  SDL_NAME(XvAdaptorInfo)*          /* adaptors */
#endif
);

extern void SDL_NAME(XvFreeEncodingInfo)(
#if NeedFunctionPrototypes
  SDL_NAME(XvEncodingInfo)*         /* encodings */
#endif
);


extern SDL_NAME(XvImageFormatValues) * SDL_NAME(XvListImageFormats) (
#if NeedFunctionPrototypes
   Display 	*display,
   XvPortID 	port_id,
   int 		*count_return
#endif
);

extern SDL_NAME(XvImage) * SDL_NAME(XvCreateImage) (
#if NeedFunctionPrototypes
   Display *display,
   XvPortID port,
   int id,
   char *data,
   int width, 
   int height 
#endif
);

extern int SDL_NAME(XvPutImage) (
#if NeedFunctionPrototypes
  Display *display,
   XvPortID id,
   Drawable d,
   GC gc,
   SDL_NAME(XvImage) *image,
   int src_x,
   int src_y,
   unsigned int src_w,
   unsigned int src_h,
   int dest_x, 
   int dest_y,
   unsigned int dest_w,
   unsigned int dest_h
#endif
);

extern int SDL_NAME(XvShmPutImage) (
#if NeedFunctionPrototypes
   Display *display,
   XvPortID id,
   Drawable d,
   GC gc,
   SDL_NAME(XvImage) *image,
   int src_x,
   int src_y,
   unsigned int src_w,
   unsigned int src_h,
   int dest_x, 
   int dest_y,
   unsigned int dest_w,
   unsigned int dest_h,
   Bool send_event
#endif
);

#ifdef _XSHM_H_

extern SDL_NAME(XvImage) * SDL_NAME(XvShmCreateImage) (
#if NeedFunctionPrototypes
   Display *display,
   XvPortID port,
   int id,
   char* data,
   int width, 
   int height,
   XShmSegmentInfo *shminfo
#endif
);

#endif


_XFUNCPROTOEND

#endif /* XVLIB_H */
