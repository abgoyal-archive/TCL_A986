

#ifndef __CRYPT_HMAC_H__
#define __CRYPT_HMAC_H__

#ifdef CRYPT_TESTPLAN
#include "crypt_testplan.h"
#else
#include "rt_config.h"
#endif /* CRYPT_TESTPLAN */

#ifdef SHA1_SUPPORT
#define HMAC_SHA1_SUPPORT
void HMAC_SHA1(IN const u8 Key[],
	       u32 KeyLen,
	       IN const u8 Message[],
	       u32 MessageLen, u8 MAC[], u32 MACLen);
#endif /* SHA1_SUPPORT */

#ifdef MD5_SUPPORT
#define HMAC_MD5_SUPPORT
void HMAC_MD5(IN const u8 Key[],
	      u32 KeyLen,
	      IN const u8 Message[],
	      u32 MessageLen, u8 MAC[], u32 MACLen);
#endif /* MD5_SUPPORT */

#endif /* __CRYPT_HMAC_H__ */
