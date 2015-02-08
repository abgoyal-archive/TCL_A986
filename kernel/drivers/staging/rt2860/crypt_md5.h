


#ifndef __CRYPT_MD5_H__
#define __CRYPT_MD5_H__

#ifdef CRYPT_TESTPLAN
#include "crypt_testplan.h"
#else
#include "rt_config.h"
#endif /* CRYPT_TESTPLAN */

/* Algorithm options */
#define MD5_SUPPORT

#ifdef MD5_SUPPORT
#define MD5_BLOCK_SIZE    64	/* 512 bits = 64 bytes */
#define MD5_DIGEST_SIZE   16	/* 128 bits = 16 bytes */

struct rt_md5_ctx_struc {
	u32 HashValue[4];
	u64 MessageLen;
	u8 Block[MD5_BLOCK_SIZE];
	u32 BlockLen;
};

void MD5_Init(struct rt_md5_ctx_struc *pMD5_CTX);
void MD5_Hash(struct rt_md5_ctx_struc *pMD5_CTX);
void MD5_Append(struct rt_md5_ctx_struc *pMD5_CTX,
		IN const u8 Message[], u32 MessageLen);
void MD5_End(struct rt_md5_ctx_struc *pMD5_CTX, u8 DigestMessage[]);
void RT_MD5(IN const u8 Message[],
	    u32 MessageLen, u8 DigestMessage[]);
#endif /* MD5_SUPPORT */

#endif /* __CRYPT_MD5_H__ */
