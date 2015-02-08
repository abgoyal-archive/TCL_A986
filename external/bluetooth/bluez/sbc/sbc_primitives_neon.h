
#ifndef __SBC_PRIMITIVES_NEON_H
#define __SBC_PRIMITIVES_NEON_H

#include "sbc_primitives.h"

#if defined(__GNUC__) && defined(__ARM_NEON__) && \
		!defined(SBC_HIGH_PRECISION) && (SCALE_OUT_BITS == 15)

#define SBC_BUILD_WITH_NEON_SUPPORT

void sbc_init_primitives_neon(struct sbc_encoder_state *encoder_state);

#endif

#endif
