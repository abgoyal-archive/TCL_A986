


#ifndef DETOKENIZE_H
#define DETOKENIZE_H

#include "onyxd_int.h"

#if ARCH_ARM
#include "arm/detokenize_arm.h"
#endif

void vp8_reset_mb_tokens_context(MACROBLOCKD *x);
int vp8_decode_mb_tokens(VP8D_COMP *, MACROBLOCKD *);

#endif /* DETOKENIZE_H */
