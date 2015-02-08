


#ifndef __CRYPT_SHA2_H__
#define __CRYPT_SHA2_H__

#ifdef CRYPT_TESTPLAN
#include "crypt_testplan.h"
#else
#include "rt_config.h"
#endif /* CRYPT_TESTPLAN */

/* Algorithm options */
#define SHA1_SUPPORT

#ifdef SHA1_SUPPORT
#define SHA1_BLOCK_SIZE    64	/* 512 bits = 64 bytes */
#define SHA1_DIGEST_SIZE   20	/* 160 bits = 20 bytes */
struct rt_sha1_ctx {
	u32 HashValue[5];	/* 5 = (SHA1_DIGEST_SIZE / 32) */
	u64 MessageLen;	/* total size */
	u8 Block[SHA1_BLOCK_SIZE];
	u32 BlockLen;
};

void RT_SHA1_Init(struct rt_sha1_ctx *pSHA_CTX);
void SHA1_Hash(struct rt_sha1_ctx *pSHA_CTX);
void SHA1_Append(struct rt_sha1_ctx *pSHA_CTX,
		 IN const u8 Message[], u32 MessageLen);
void SHA1_End(struct rt_sha1_ctx *pSHA_CTX, u8 DigestMessage[]);
void RT_SHA1(IN const u8 Message[],
	     u32 MessageLen, u8 DigestMessage[]);
#endif /* SHA1_SUPPORT */

#endif /* __CRYPT_SHA2_H__ */
