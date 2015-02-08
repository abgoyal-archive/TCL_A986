


#ifndef common_h
#define common_h 1

#include <assert.h>

/* Interface header for common constant data structures and lookup tables */

#include "vpx_mem/vpx_mem.h"

#include "common_types.h"

/* Only need this for fixed-size arrays, for structs just assign. */

#define vp8_copy( Dest, Src) { \
        assert( sizeof( Dest) == sizeof( Src)); \
        vpx_memcpy( Dest, Src, sizeof( Src)); \
    }

/* Use this for variably-sized arrays. */

#define vp8_copy_array( Dest, Src, N) { \
        assert( sizeof( *Dest) == sizeof( *Src)); \
        vpx_memcpy( Dest, Src, N * sizeof( *Src)); \
    }

#define vp8_zero( Dest)  vpx_memset( &Dest, 0, sizeof( Dest));

#define vp8_zero_array( Dest, N)  vpx_memset( Dest, 0, N * sizeof( *Dest));


#endif  /* common_h */
