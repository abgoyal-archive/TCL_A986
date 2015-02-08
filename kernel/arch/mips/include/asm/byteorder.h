
#ifndef _ASM_BYTEORDER_H
#define _ASM_BYTEORDER_H

#if defined(__MIPSEB__)
#include <linux/byteorder/big_endian.h>
#elif defined(__MIPSEL__)
#include <linux/byteorder/little_endian.h>
#else
# error "MIPS, but neither __MIPSEB__, nor __MIPSEL__???"
#endif

#endif /* _ASM_BYTEORDER_H */
