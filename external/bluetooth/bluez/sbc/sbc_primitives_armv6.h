
#ifndef __SBC_PRIMITIVES_ARMV6_H
#define __SBC_PRIMITIVES_ARMV6_H

#include "sbc_primitives.h"

#if defined(__ARM_ARCH_6__) || defined(__ARM_ARCH_6J__) || \
	defined(__ARM_ARCH_6K__) || defined(__ARM_ARCH_6Z__) || \
	defined(__ARM_ARCH_6ZK__) || defined(__ARM_ARCH_6T2__) || \
	defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_7__) || \
	defined(__ARM_ARCH_7A__) || defined(__ARM_ARCH_7R__) || \
	defined(__ARM_ARCH_7M__)
#define SBC_HAVE_ARMV6 1
#endif

#if !defined(SBC_HIGH_PRECISION) && (SCALE_OUT_BITS == 15) && \
	defined(__GNUC__) && defined(SBC_HAVE_ARMV6) && \
	defined(__ARM_EABI__) && !defined(__thumb__) && \
	!defined(__ARM_NEON__)

#define SBC_BUILD_WITH_ARMV6_SUPPORT

void sbc_init_primitives_armv6(struct sbc_encoder_state *encoder_state);

#endif

#endif
