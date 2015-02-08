

#ifndef _CRYPTO_INTERNAL_RNG_H
#define _CRYPTO_INTERNAL_RNG_H

#include <crypto/algapi.h>
#include <crypto/rng.h>

extern const struct crypto_type crypto_rng_type;

static inline void *crypto_rng_ctx(struct crypto_rng *tfm)
{
	return crypto_tfm_ctx(&tfm->base);
}

#endif
