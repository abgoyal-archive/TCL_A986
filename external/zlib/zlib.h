

#ifndef ZLIB_H
#define ZLIB_H

#include "zconf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ZLIB_VERSION "1.2.5"
#define ZLIB_VERNUM 0x1250
#define ZLIB_VER_MAJOR 1
#define ZLIB_VER_MINOR 2
#define ZLIB_VER_REVISION 5
#define ZLIB_VER_SUBREVISION 0


typedef voidpf (*alloc_func) OF((voidpf opaque, uInt items, uInt size));
typedef void   (*free_func)  OF((voidpf opaque, voidpf address));

struct internal_state;

typedef struct z_stream_s {
    Bytef    *next_in;  /* next input byte */
    uInt     avail_in;  /* number of bytes available at next_in */
    uLong    total_in;  /* total nb of input bytes read so far */

    Bytef    *next_out; /* next output byte should be put there */
    uInt     avail_out; /* remaining free space at next_out */
    uLong    total_out; /* total nb of bytes output so far */

    char     *msg;      /* last error message, NULL if no error */
    struct internal_state FAR *state; /* not visible by applications */

    alloc_func zalloc;  /* used to allocate the internal state */
    free_func  zfree;   /* used to free the internal state */
    voidpf     opaque;  /* private data object passed to zalloc and zfree */

    int     data_type;  /* best guess about the data type: binary or text */
    uLong   adler;      /* adler32 value of the uncompressed data */
    uLong   reserved;   /* reserved for future use */
} z_stream;

typedef z_stream FAR *z_streamp;

typedef struct gz_header_s {
    int     text;       /* true if compressed data believed to be text */
    uLong   time;       /* modification time */
    int     xflags;     /* extra flags (not used when writing a gzip file) */
    int     os;         /* operating system */
    Bytef   *extra;     /* pointer to extra field or Z_NULL if none */
    uInt    extra_len;  /* extra field length (valid if extra != Z_NULL) */
    uInt    extra_max;  /* space at extra (only when reading header) */
    Bytef   *name;      /* pointer to zero-terminated file name or Z_NULL */
    uInt    name_max;   /* space at name (only when reading header) */
    Bytef   *comment;   /* pointer to zero-terminated comment or Z_NULL */
    uInt    comm_max;   /* space at comment (only when reading header) */
    int     hcrc;       /* true if there was or will be a header crc */
    int     done;       /* true when done reading gzip header (not used
                           when writing a gzip file) */
} gz_header;

typedef gz_header FAR *gz_headerp;


                        /* constants */

#define Z_NO_FLUSH      0
#define Z_PARTIAL_FLUSH 1
#define Z_SYNC_FLUSH    2
#define Z_FULL_FLUSH    3
#define Z_FINISH        4
#define Z_BLOCK         5
#define Z_TREES         6
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
#define Z_RLE                 3
#define Z_FIXED               4
#define Z_DEFAULT_STRATEGY    0
/* compression strategy; see deflateInit2() below for details */

#define Z_BINARY   0
#define Z_TEXT     1
#define Z_ASCII    Z_TEXT   /* for compatibility with 1.2.2 and earlier */
#define Z_UNKNOWN  2
/* Possible values of the data_type field (though see inflate()) */

#define Z_DEFLATED   8
/* The deflate compression method (the only one supported in this version) */

#define Z_NULL  0  /* for initializing zalloc, zfree, opaque */

#define zlib_version zlibVersion()
/* for compatibility with versions < 1.0.2 */


                        /* basic functions */

ZEXTERN const char * ZEXPORT zlibVersion OF((void));



ZEXTERN int ZEXPORT deflate OF((z_streamp strm, int flush));


ZEXTERN int ZEXPORT deflateEnd OF((z_streamp strm));




ZEXTERN int ZEXPORT inflate OF((z_streamp strm, int flush));


ZEXTERN int ZEXPORT inflateEnd OF((z_streamp strm));


                        /* Advanced functions */



ZEXTERN int ZEXPORT deflateSetDictionary OF((z_streamp strm,
                                             const Bytef *dictionary,
                                             uInt  dictLength));

ZEXTERN int ZEXPORT deflateCopy OF((z_streamp dest,
                                    z_streamp source));

ZEXTERN int ZEXPORT deflateReset OF((z_streamp strm));

ZEXTERN int ZEXPORT deflateParams OF((z_streamp strm,
                                      int level,
                                      int strategy));

ZEXTERN int ZEXPORT deflateTune OF((z_streamp strm,
                                    int good_length,
                                    int max_lazy,
                                    int nice_length,
                                    int max_chain));

ZEXTERN uLong ZEXPORT deflateBound OF((z_streamp strm,
                                       uLong sourceLen));

ZEXTERN int ZEXPORT deflatePrime OF((z_streamp strm,
                                     int bits,
                                     int value));

ZEXTERN int ZEXPORT deflateSetHeader OF((z_streamp strm,
                                         gz_headerp head));


ZEXTERN int ZEXPORT inflateSetDictionary OF((z_streamp strm,
                                             const Bytef *dictionary,
                                             uInt  dictLength));

ZEXTERN int ZEXPORT inflateSync OF((z_streamp strm));

ZEXTERN int ZEXPORT inflateCopy OF((z_streamp dest,
                                    z_streamp source));

ZEXTERN int ZEXPORT inflateReset OF((z_streamp strm));

ZEXTERN int ZEXPORT inflateReset2 OF((z_streamp strm,
                                      int windowBits));

ZEXTERN int ZEXPORT inflatePrime OF((z_streamp strm,
                                     int bits,
                                     int value));

ZEXTERN long ZEXPORT inflateMark OF((z_streamp strm));

ZEXTERN int ZEXPORT inflateGetHeader OF((z_streamp strm,
                                         gz_headerp head));


typedef unsigned (*in_func) OF((void FAR *, unsigned char FAR * FAR *));
typedef int (*out_func) OF((void FAR *, unsigned char FAR *, unsigned));

ZEXTERN int ZEXPORT inflateBack OF((z_streamp strm,
                                    in_func in, void FAR *in_desc,
                                    out_func out, void FAR *out_desc));

ZEXTERN int ZEXPORT inflateBackEnd OF((z_streamp strm));

ZEXTERN uLong ZEXPORT zlibCompileFlags OF((void));


                        /* utility functions */


ZEXTERN int ZEXPORT compress OF((Bytef *dest,   uLongf *destLen,
                                 const Bytef *source, uLong sourceLen));

ZEXTERN int ZEXPORT compress2 OF((Bytef *dest,   uLongf *destLen,
                                  const Bytef *source, uLong sourceLen,
                                  int level));

ZEXTERN uLong ZEXPORT compressBound OF((uLong sourceLen));

ZEXTERN int ZEXPORT uncompress OF((Bytef *dest,   uLongf *destLen,
                                   const Bytef *source, uLong sourceLen));


                        /* gzip file access functions */


typedef voidp gzFile;       /* opaque gzip file descriptor */


ZEXTERN gzFile ZEXPORT gzdopen OF((int fd, const char *mode));

ZEXTERN int ZEXPORT gzbuffer OF((gzFile file, unsigned size));

ZEXTERN int ZEXPORT gzsetparams OF((gzFile file, int level, int strategy));

ZEXTERN int ZEXPORT gzread OF((gzFile file, voidp buf, unsigned len));

ZEXTERN int ZEXPORT gzwrite OF((gzFile file,
                                voidpc buf, unsigned len));

ZEXTERN int ZEXPORTVA gzprintf OF((gzFile file, const char *format, ...));

ZEXTERN int ZEXPORT gzputs OF((gzFile file, const char *s));

ZEXTERN char * ZEXPORT gzgets OF((gzFile file, char *buf, int len));

ZEXTERN int ZEXPORT gzputc OF((gzFile file, int c));

ZEXTERN int ZEXPORT gzgetc OF((gzFile file));

ZEXTERN int ZEXPORT gzungetc OF((int c, gzFile file));

ZEXTERN int ZEXPORT gzflush OF((gzFile file, int flush));


ZEXTERN int ZEXPORT    gzrewind OF((gzFile file));



ZEXTERN int ZEXPORT gzeof OF((gzFile file));

ZEXTERN int ZEXPORT gzdirect OF((gzFile file));

ZEXTERN int ZEXPORT    gzclose OF((gzFile file));

ZEXTERN int ZEXPORT gzclose_r OF((gzFile file));
ZEXTERN int ZEXPORT gzclose_w OF((gzFile file));

ZEXTERN const char * ZEXPORT gzerror OF((gzFile file, int *errnum));

ZEXTERN void ZEXPORT gzclearerr OF((gzFile file));


                        /* checksum functions */


ZEXTERN uLong ZEXPORT adler32 OF((uLong adler, const Bytef *buf, uInt len));


ZEXTERN uLong ZEXPORT crc32   OF((uLong crc, const Bytef *buf, uInt len));



                        /* various hacks, don't look :) */

ZEXTERN int ZEXPORT deflateInit_ OF((z_streamp strm, int level,
                                     const char *version, int stream_size));
ZEXTERN int ZEXPORT inflateInit_ OF((z_streamp strm,
                                     const char *version, int stream_size));
ZEXTERN int ZEXPORT deflateInit2_ OF((z_streamp strm, int  level, int  method,
                                      int windowBits, int memLevel,
                                      int strategy, const char *version,
                                      int stream_size));
ZEXTERN int ZEXPORT inflateInit2_ OF((z_streamp strm, int  windowBits,
                                      const char *version, int stream_size));
ZEXTERN int ZEXPORT inflateBackInit_ OF((z_streamp strm, int windowBits,
                                         unsigned char FAR *window,
                                         const char *version,
                                         int stream_size));
#define deflateInit(strm, level) \
        deflateInit_((strm), (level),       ZLIB_VERSION, sizeof(z_stream))
#define inflateInit(strm) \
        inflateInit_((strm),                ZLIB_VERSION, sizeof(z_stream))
#define deflateInit2(strm, level, method, windowBits, memLevel, strategy) \
        deflateInit2_((strm),(level),(method),(windowBits),(memLevel),\
                      (strategy),           ZLIB_VERSION, sizeof(z_stream))
#define inflateInit2(strm, windowBits) \
        inflateInit2_((strm), (windowBits), ZLIB_VERSION, sizeof(z_stream))
#define inflateBackInit(strm, windowBits, window) \
        inflateBackInit_((strm), (windowBits), (window), \
                                            ZLIB_VERSION, sizeof(z_stream))

#if defined(_LARGEFILE64_SOURCE) && _LFS64_LARGEFILE-0
   ZEXTERN gzFile ZEXPORT gzopen64 OF((const char *, const char *));
   ZEXTERN z_off64_t ZEXPORT gzseek64 OF((gzFile, z_off64_t, int));
   ZEXTERN z_off64_t ZEXPORT gztell64 OF((gzFile));
   ZEXTERN z_off64_t ZEXPORT gzoffset64 OF((gzFile));
   ZEXTERN uLong ZEXPORT adler32_combine64 OF((uLong, uLong, z_off64_t));
   ZEXTERN uLong ZEXPORT crc32_combine64 OF((uLong, uLong, z_off64_t));
#endif

#if !defined(ZLIB_INTERNAL) && _FILE_OFFSET_BITS-0 == 64 && _LFS64_LARGEFILE-0
#  define gzopen gzopen64
#  define gzseek gzseek64
#  define gztell gztell64
#  define gzoffset gzoffset64
#  define adler32_combine adler32_combine64
#  define crc32_combine crc32_combine64
#  ifdef _LARGEFILE64_SOURCE
     ZEXTERN gzFile ZEXPORT gzopen64 OF((const char *, const char *));
     ZEXTERN z_off_t ZEXPORT gzseek64 OF((gzFile, z_off_t, int));
     ZEXTERN z_off_t ZEXPORT gztell64 OF((gzFile));
     ZEXTERN z_off_t ZEXPORT gzoffset64 OF((gzFile));
     ZEXTERN uLong ZEXPORT adler32_combine64 OF((uLong, uLong, z_off_t));
     ZEXTERN uLong ZEXPORT crc32_combine64 OF((uLong, uLong, z_off_t));
#  endif
#else
   ZEXTERN gzFile ZEXPORT gzopen OF((const char *, const char *));
   ZEXTERN z_off_t ZEXPORT gzseek OF((gzFile, z_off_t, int));
   ZEXTERN z_off_t ZEXPORT gztell OF((gzFile));
   ZEXTERN z_off_t ZEXPORT gzoffset OF((gzFile));
   ZEXTERN uLong ZEXPORT adler32_combine OF((uLong, uLong, z_off_t));
   ZEXTERN uLong ZEXPORT crc32_combine OF((uLong, uLong, z_off_t));
#endif

/* hack for buggy compilers */
#if !defined(ZUTIL_H) && !defined(NO_DUMMY_DECL)
    struct internal_state {int dummy;};
#endif

/* undocumented functions */
ZEXTERN const char   * ZEXPORT zError           OF((int));
ZEXTERN int            ZEXPORT inflateSyncPoint OF((z_streamp));
ZEXTERN const uLongf * ZEXPORT get_crc_table    OF((void));
ZEXTERN int            ZEXPORT inflateUndermine OF((z_streamp, int));

#ifdef __cplusplus
}
#endif

#endif /* ZLIB_H */
