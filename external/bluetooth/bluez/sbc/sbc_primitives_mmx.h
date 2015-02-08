
#ifndef __SBC_PRIMITIVES_MMX_H
#define __SBC_PRIMITIVES_MMX_H

#include "sbc_primitives.h"

#if defined(__GNUC__) && (defined(__i386__) || defined(__amd64__)) && \
		!defined(SBC_HIGH_PRECISION) && (SCALE_OUT_BITS == 15)

#define SBC_BUILD_WITH_MMX_SUPPORT

void sbc_init_primitives_mmx(struct sbc_encoder_state *encoder_state);

#endif

#endif
