


/* @(#) $Id: zutil.h,v 1.1 2000/01/01 03:32:23 davem Exp $ */

#ifndef _Z_UTIL_H
#define _Z_UTIL_H

#include <linux/zlib.h>
#include <linux/string.h>
#include <linux/kernel.h>

typedef unsigned char  uch;
typedef unsigned short ush;
typedef unsigned long  ulg;

        /* common constants */

#define STORED_BLOCK 0
#define STATIC_TREES 1
#define DYN_TREES    2
/* The three kinds of block type */

#define MIN_MATCH  3
#define MAX_MATCH  258
/* The minimum and maximum match lengths */

#define PRESET_DICT 0x20 /* preset dictionary flag in zlib header */

        /* target dependencies */

        /* Common defaults */

#ifndef OS_CODE
#  define OS_CODE  0x03  /* assume Unix */
#endif

         /* functions */

typedef uLong (*check_func) (uLong check, const Byte *buf,
				       uInt len);


                        /* checksum functions */

#define BASE 65521L /* largest prime smaller than 65536 */
#define NMAX 5552
/* NMAX is the largest n such that 255n(n+1)/2 + (n+1)(BASE-1) <= 2^32-1 */

#define DO1(buf,i)  {s1 += buf[i]; s2 += s1;}
#define DO2(buf,i)  DO1(buf,i); DO1(buf,i+1);
#define DO4(buf,i)  DO2(buf,i); DO2(buf,i+2);
#define DO8(buf,i)  DO4(buf,i); DO4(buf,i+4);
#define DO16(buf)   DO8(buf,0); DO8(buf,8);

/* ========================================================================= */
static inline uLong zlib_adler32(uLong adler,
				 const Byte *buf,
				 uInt len)
{
    unsigned long s1 = adler & 0xffff;
    unsigned long s2 = (adler >> 16) & 0xffff;
    int k;

    if (buf == NULL) return 1L;

    while (len > 0) {
        k = len < NMAX ? len : NMAX;
        len -= k;
        while (k >= 16) {
            DO16(buf);
	    buf += 16;
            k -= 16;
        }
        if (k != 0) do {
            s1 += *buf++;
	    s2 += s1;
        } while (--k);
        s1 %= BASE;
        s2 %= BASE;
    }
    return (s2 << 16) | s1;
}

#endif /* _Z_UTIL_H */
