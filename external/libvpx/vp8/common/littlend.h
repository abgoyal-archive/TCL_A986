


#ifndef _littlend_h
#define _littlend_h

#if defined(__cplusplus)
extern "C" {
#endif

#define invert2(x) (x)
#define invert4(x) (x)

#define low_byte(x) (unsigned char)x
#define mid1Byte(x) (unsigned char)(x >> 8)
#define mid2Byte(x) (unsigned char)(x >> 16)
#define high_byte(x) (unsigned char)(x >> 24)

#define SWAPENDS 0

#if defined(__cplusplus)
}
#endif

#endif
