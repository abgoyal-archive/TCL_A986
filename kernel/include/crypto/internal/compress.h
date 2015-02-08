

#ifndef _CRYPTO_INTERNAL_COMPRESS_H
#define _CRYPTO_INTERNAL_COMPRESS_H

#include <crypto/compress.h>

extern int crypto_register_pcomp(struct pcomp_alg *alg);
extern int crypto_unregister_pcomp(struct pcomp_alg *alg);

#endif	/* _CRYPTO_INTERNAL_COMPRESS_H */
