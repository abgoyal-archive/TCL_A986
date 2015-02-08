
/* @(#) $Id$ */

#ifndef _ZCONF_H
#define _ZCONF_H


/* Maximum value for memLevel in deflateInit2 */
#ifndef MAX_MEM_LEVEL
#  define MAX_MEM_LEVEL 8
#endif

#ifndef MAX_WBITS
#  define MAX_WBITS   15 /* 32K LZ77 window */
#endif

/* default windowBits for decompression. MAX_WBITS is for compression only */
#ifndef DEF_WBITS
#  define DEF_WBITS MAX_WBITS
#endif

/* default memLevel */
#if MAX_MEM_LEVEL >= 8
#  define DEF_MEM_LEVEL 8
#else
#  define DEF_MEM_LEVEL  MAX_MEM_LEVEL
#endif

                        /* Type declarations */

typedef unsigned char  Byte;  /* 8 bits */
typedef unsigned int   uInt;  /* 16 bits or more */
typedef unsigned long  uLong; /* 32 bits or more */
typedef void     *voidp;

#endif /* _ZCONF_H */
