
#ifndef _qemu_cbuffer_h
#define _qemu_cbuffer_h

#include <stdint.h>

/* Basic circular buffer type and methods */

typedef struct {
    uint8_t*  buff;
    int       size;
    int       rpos;
    int       count;
} CBuffer;

static __inline__ void
cbuffer_reset( CBuffer*  cb, void*  buff, int  size )
{
    cb->buff  = buff;
    cb->size  = size;
    cb->rpos  = 0;
    cb->count = 0;
}

static __inline__ int
cbuffer_write_avail( CBuffer*  cb )
{
    return cb->size - cb->count;
}

extern int  cbuffer_write( CBuffer*  cb, const void*  from, int  len );
extern int  cbuffer_write_peek( CBuffer*  cb, uint8_t*  *pbase );
extern void cbuffer_write_step( CBuffer*  cb, int  len );

static __inline__ int
cbuffer_read_avail( CBuffer*  cb )
{
    return cb->count;
}

extern int  cbuffer_read( CBuffer*  cb, void*  to, int  len );
extern int  cbuffer_read_peek( CBuffer*  cb, uint8_t*  *pbase );
extern void cbuffer_read_step( CBuffer*  cb, int  len );

extern const char*  cbuffer_quote( CBuffer*  cb );
extern const char*  cbuffer_quote_data( CBuffer*  cb );
extern void         cbuffer_print( CBuffer*  cb );

#endif /* qemu_cbuffer_h */


