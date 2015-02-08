
#ifndef _CRYPTO_AUTHENC_H
#define _CRYPTO_AUTHENC_H

#include <linux/types.h>

enum {
	CRYPTO_AUTHENC_KEYA_UNSPEC,
	CRYPTO_AUTHENC_KEYA_PARAM,
};

struct crypto_authenc_key_param {
	__be32 enckeylen;
};

#endif	/* _CRYPTO_AUTHENC_H */

