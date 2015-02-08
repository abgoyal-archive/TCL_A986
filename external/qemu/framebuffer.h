
#ifndef _QEMU_FRAMEBUFFER_H_
#define _QEMU_FRAMEBUFFER_H_

typedef struct QFrameBuffer   QFrameBuffer;


typedef enum {
    QFRAME_BUFFER_NONE   = 0,
    QFRAME_BUFFER_RGB565 = 1,
    QFRAME_BUFFER_MAX          /* do not remove */
} QFrameBufferFormat;

struct QFrameBuffer {
    int                 width;        /* width in pixels */
    int                 height;       /* height in pixels */
    int                 pitch;        /* bytes per line */
    int                 rotation;     /* rotation to be applied when displaying */
    QFrameBufferFormat  format;
    void*               pixels;       /* pixel buffer */

    int                 phys_width_mm;
    int                 phys_height_mm;

    /* extra data that is handled by the framebuffer implementation */
    void*               extra;

};

#define  DEFAULT_FRAMEBUFFER_DPI   165


/* initialize a framebuffer object and allocate its pixel buffer */
/* this computes phys_width_mm and phys_height_mm assuming a 165 dpi screen */
/* returns -1 in case of error, 0 otherwise */
extern int
qframebuffer_init( QFrameBuffer*       qfbuff,
                   int                 width,
                   int                 height,
                   int                 rotation,
                   QFrameBufferFormat  format );

extern void
qframebuffer_set_dpi( QFrameBuffer*   qfbuff,
                      int             x_dpi,
                      int             y_dpi );

extern void
qframebuffer_set_mm( QFrameBuffer*   qfbuff,
                     int             width_mm,
                     int             height_mm );

typedef void (*QFrameBufferUpdateFunc)( void*  opaque, int  x, int  y, 
                                                       int  w, int  h );

typedef void (*QFrameBufferRotateFunc)( void*  opaque, int  rotation );

typedef void (*QFrameBufferDoneFunc)  ( void*  opaque );

extern void
qframebuffer_add_client( QFrameBuffer*           qfbuff,
                         void*                   fb_opaque,
                         QFrameBufferUpdateFunc  fb_update,
                         QFrameBufferRotateFunc  fb_rotate,
                         QFrameBufferDoneFunc    fb_done );

typedef void (*QFrameBufferCheckUpdateFunc)( void*  opaque );

typedef void (*QFrameBufferInvalidateFunc) ( void*  opaque );

typedef void (*QFrameBufferDetachFunc)     ( void*  opaque );

/* set the producer of a given framebuffer */
extern void
qframebuffer_set_producer( QFrameBuffer*                qfbuff,
                           void*                        opaque,
                           QFrameBufferCheckUpdateFunc  fb_check,
                           QFrameBufferInvalidateFunc   fb_invalidate,
                           QFrameBufferDetachFunc       fb_detach );

extern void
qframebuffer_update( QFrameBuffer*  qfbuff, int  x, int  y, int  w, int  h );

extern void
qframebuffer_rotate( QFrameBuffer*  qfbuff, int  rotation );

extern void
qframebuffer_done( QFrameBuffer*   qfbuff );


extern void
qframebuffer_check_updates( void );

extern void
qframebuffer_invalidate_all( void );


/* add a new constructed frame buffer object to our global list */
extern void
qframebuffer_fifo_add( QFrameBuffer*  qfbuff );

/* retrieve a frame buffer object from the global FIFO list */
extern QFrameBuffer*
qframebuffer_fifo_get( void );

/* */

#endif /* _QEMU_FRAMEBUFFER_H_ */

