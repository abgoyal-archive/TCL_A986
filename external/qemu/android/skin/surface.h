
#ifndef _ANDROID_SKIN_SURFACE_H
#define _ANDROID_SKIN_SURFACE_H

#include "android/skin/region.h"
#include <stdint.h>

typedef struct SkinSurface  SkinSurface;

/* increment surface's reference count */
extern SkinSurface*  skin_surface_ref( SkinSurface*  surface );

extern void          skin_surface_unrefp( SkinSurface*  *psurface );

typedef void (*SkinSurfaceDoneFunc)( void*  user );

extern void  skin_surface_set_done( SkinSurface*  s, SkinSurfaceDoneFunc  done_func, void*  done_user );



/* create a 'fast' surface that contains a copy of an input ARGB32 pixmap */
extern SkinSurface*  skin_surface_create_fast( int  w, int  h );

/* create an empty 'slow' surface containing an ARGB32 pixmap */
extern SkinSurface*  skin_surface_create_slow( int  w, int  h );

extern SkinSurface*  skin_surface_create_argb32_from(
                            int                  w,
                            int                  h,
                            int                  pitch,
                            uint32_t*            pixels,
                            int                  do_copy );

/* surface pixels information for slow surfaces */
typedef struct {
    int         w;
    int         h;
    int         pitch;
    uint32_t*   pixels;
} SkinSurfacePixels;

extern int     skin_surface_lock  ( SkinSurface*  s, SkinSurfacePixels  *pix );

/* unlock a slow surface that was previously locked */
extern void    skin_surface_unlock( SkinSurface*  s );

/* list of composition operators for the blit routine */
typedef enum {
    SKIN_BLIT_COPY = 0,
    SKIN_BLIT_SRCOVER,
    SKIN_BLIT_DSTOVER,
} SkinBlitOp;


/* blit a surface into another one */
extern void    skin_surface_blit( SkinSurface*  dst,
                                  SkinPos*      dst_pos,
                                  SkinSurface*  src,
                                  SkinRect*     src_rect,
                                  SkinBlitOp    blitop );

/* blit a colored rectangle into a destination surface */
extern void    skin_surface_fill( SkinSurface*  dst,
                                  SkinRect*     rect,
                                  uint32_t      argb_premul,
                                  SkinBlitOp    blitop );

#endif /* _ANDROID_SKIN_SURFACE_H */
