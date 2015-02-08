
#ifndef _SHA_256_H_
#define _SHA_256_H_

#include "base/third_party/nspr/prtypes.h"

struct SHA256ContextStr {
    union {
	PRUint32 w[64];	    /* message schedule, input buffer, plus 48 words */
	PRUint8  b[256];
    } u;
    PRUint32 h[8];		/* 8 state variables */
    PRUint32 sizeHi,sizeLo;	/* 64-bit count of hashed bytes. */
};

#endif /* _SHA_256_H_ */
