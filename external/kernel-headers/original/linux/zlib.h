
#ifndef _ZLIB_H
#define _ZLIB_H

#include <linux/zconf.h>

/* zlib deflate based on ZLIB_VERSION "1.1.3" */
/* zlib inflate based on ZLIB_VERSION "1.2.3" */



struct internal_state;

typedef struct z_stream_s {
    Byte    *next_in;   /* next input byte */
    uInt     avail_in;  /* number of bytes available at next_in */
    uLong    total_in;  /* total nb of input bytes read so far */

    Byte    *next_out;  /* next output byte should be put there */
    uInt     avail_out; /* remaining free space at next_out */
    uLong    total_out; /* total nb of bytes output so far */

    char     *msg;      /* last error message, NULL if no error */
    struct internal_state *state; /* not visible by applications */

    void     *workspace; /* memory allocated for this stream */

    int     data_type;  /* best guess about the data type: ascii or binary */
    uLong   adler;      /* adler32 value of the uncompressed data */
    uLong   reserved;   /* reserved for future use */
} z_stream;

typedef z_stream *z_streamp;


                        /* constants */

#define Z_NO_FLUSH      0
#define Z_PARTIAL_FLUSH 1 /* will be removed, use Z_SYNC_FLUSH instead */
#define Z_PACKET_FLUSH  2
#define Z_SYNC_FLUSH    3
#define Z_FULL_FLUSH    4
#define Z_FINISH        5
#define Z_BLOCK         6 /* Only for inflate at present */
/* Allowed flush values; see deflate() and inflate() below for details */

#define Z_OK            0
#define Z_STREAM_END    1
#define Z_NEED_DICT     2
#define Z_ERRNO        (-1)
#define Z_STREAM_ERROR (-2)
#define Z_DATA_ERROR   (-3)
#define Z_MEM_ERROR    (-4)
#define Z_BUF_ERROR    (-5)
#define Z_VERSION_ERROR (-6)

#define Z_NO_COMPRESSION         0
#define Z_BEST_SPEED             1
#define Z_BEST_COMPRESSION       9
#define Z_DEFAULT_COMPRESSION  (-1)
/* compression levels */

#define Z_FILTERED            1
#define Z_HUFFMAN_ONLY        2
#define Z_DEFAULT_STRATEGY    0
/* compression strategy; see deflateInit2() below for details */

#define Z_BINARY   0
#define Z_ASCII    1
#define Z_UNKNOWN  2
/* Possible values of the data_type field */

#define Z_DEFLATED   8
/* The deflate compression method (the only one supported in this version) */

                        /* basic functions */

extern int zlib_deflate_workspacesize (void);



extern int zlib_deflate (z_streamp strm, int flush);


extern int zlib_deflateEnd (z_streamp strm);


extern int zlib_inflate_workspacesize (void);



extern int zlib_inflate (z_streamp strm, int flush);


extern int zlib_inflateEnd (z_streamp strm);

                        /* Advanced functions */


                            
#if 0
extern int zlib_deflateSetDictionary (z_streamp strm,
						     const Byte *dictionary,
						     uInt  dictLength);
#endif

#if 0
extern int zlib_deflateCopy (z_streamp dest, z_streamp source);
#endif


extern int zlib_deflateReset (z_streamp strm);

static inline unsigned long deflateBound(unsigned long s)
{
	return s + ((s + 7) >> 3) + ((s + 63) >> 6) + 11;
}

#if 0
extern int zlib_deflateParams (z_streamp strm, int level, int strategy);
#endif


extern int zlib_inflateSetDictionary (z_streamp strm,
						     const Byte *dictionary,
						     uInt  dictLength);

#if 0
extern int zlib_inflateSync (z_streamp strm);
#endif

extern int zlib_inflateReset (z_streamp strm);

extern int zlib_inflateIncomp (z_stream *strm);

#define zlib_deflateInit(strm, level) \
	zlib_deflateInit2((strm), (level), Z_DEFLATED, MAX_WBITS, \
			      DEF_MEM_LEVEL, Z_DEFAULT_STRATEGY)
#define zlib_inflateInit(strm) \
	zlib_inflateInit2((strm), DEF_WBITS)

extern int zlib_deflateInit2(z_streamp strm, int  level, int  method,
                                      int windowBits, int memLevel,
                                      int strategy);
extern int zlib_inflateInit2(z_streamp strm, int  windowBits);

#if !defined(_Z_UTIL_H) && !defined(NO_DUMMY_DECL)
    struct internal_state {int dummy;}; /* hack for buggy compilers */
#endif

#endif /* _ZLIB_H */
