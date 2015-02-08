

#ifndef PPPCRYPT_H
#define	PPPCRYPT_H

#ifdef HAVE_CRYPT_H
#include <crypt.h>
#endif

#ifndef USE_CRYPT
#ifdef ANDROID_CHANGES
#include <openssl/des.h>
#else
#include <des.h>
#endif
#endif

extern bool	DesSetkey __P((u_char *));
extern bool	DesEncrypt __P((u_char *, u_char *));
extern bool	DesDecrypt __P((u_char *, u_char *));

#endif /* PPPCRYPT_H */
