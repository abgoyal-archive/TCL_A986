/*	$NetBSD: sha2.h,v 1.4 2006/09/09 16:22:36 manu Exp $	*/

/*	$KAME: sha2.h,v 1.2 2001/08/08 22:09:27 sakane Exp $	*/


#ifndef __SHA2_H__
#define __SHA2_H__

#ifdef __cplusplus
extern "C" {
#endif


/*** SHA-256/384/512 Various Length Definitions ***********************/
#define SHA256_BLOCK_LENGTH		64
#define SHA256_DIGEST_LENGTH		32
#define SHA256_DIGEST_STRING_LENGTH	(SHA256_DIGEST_LENGTH * 2 + 1)
#define SHA384_BLOCK_LENGTH		128
#define SHA384_DIGEST_LENGTH		48
#define SHA384_DIGEST_STRING_LENGTH	(SHA384_DIGEST_LENGTH * 2 + 1)
#define SHA512_BLOCK_LENGTH		128
#define SHA512_DIGEST_LENGTH		64
#define SHA512_DIGEST_STRING_LENGTH	(SHA512_DIGEST_LENGTH * 2 + 1)


/*** SHA-256/384/512 Context Structures *******************************/
#if 0
typedef unsigned char u_int8_t;		/* 1-byte  (8-bits)  */
typedef unsigned int u_int32_t;		/* 4-bytes (32-bits) */
typedef unsigned long long u_int64_t;	/* 8-bytes (64-bits) */
#endif

#ifndef HAVE_SHA2_IN_SHA_H
#if 0 /*def SHA2_USE_INTTYPES_H*/

typedef struct _SHA256_CTX {
	uint32_t	state[8];
	uint64_t	bitcount;
	uint8_t	buffer[SHA256_BLOCK_LENGTH];
} SHA256_CTX;
typedef struct _SHA512_CTX {
	uint64_t	state[8];
	uint64_t	bitcount[2];
	uint8_t	buffer[SHA512_BLOCK_LENGTH];
} SHA512_CTX;

#else /* SHA2_USE_INTTYPES_H */

typedef struct _SHA256_CTX {
	u_int32_t	state[8];
	u_int64_t	bitcount;
	u_int8_t	buffer[SHA256_BLOCK_LENGTH];
} SHA256_CTX;
typedef struct _SHA512_CTX {
	u_int64_t	state[8];
	u_int64_t	bitcount[2];
	u_int8_t	buffer[SHA512_BLOCK_LENGTH];
} SHA512_CTX;

#endif /* SHA2_USE_INTTYPES_H */
#endif /* HAVE_SHA2_IN_SHA_H */

typedef SHA512_CTX SHA384_CTX;


/*** SHA-256/384/512 Function Prototypes ******************************/

#ifndef HAVE_SHA2_IN_SHA_H
void SHA256_Init __P((SHA256_CTX *));
void SHA256_Update __P((SHA256_CTX*, const u_int8_t*, size_t));
void SHA256_Final __P((u_int8_t[SHA256_DIGEST_LENGTH], SHA256_CTX*));
#endif /* HAVE_SHA2_IN_SHA_H */
char* SHA256_End __P((SHA256_CTX*, char[SHA256_DIGEST_STRING_LENGTH]));
char* SHA256_Data __P((const u_int8_t*, size_t, char[SHA256_DIGEST_STRING_LENGTH]));

#ifndef HAVE_SHA2_IN_SHA_H
void SHA384_Init __P((SHA384_CTX*));
void SHA384_Update __P((SHA384_CTX*, const u_int8_t*, size_t));
void SHA384_Final __P((u_int8_t[SHA384_DIGEST_LENGTH], SHA384_CTX*));
#endif /* HAVE_SHA2_IN_SHA_H */
char* SHA384_End __P((SHA384_CTX*, char[SHA384_DIGEST_STRING_LENGTH]));
char* SHA384_Data __P((const u_int8_t*, size_t, char[SHA384_DIGEST_STRING_LENGTH]));

#ifndef HAVE_SHA2_IN_SHA_H
void SHA512_Init __P((SHA512_CTX*));
void SHA512_Update __P((SHA512_CTX*, const u_int8_t*, size_t));
void SHA512_Final __P((u_int8_t[SHA512_DIGEST_LENGTH], SHA512_CTX*));
#endif /* HAVE_SHA2_IN_SHA_H */
char* SHA512_End __P((SHA512_CTX*, char[SHA512_DIGEST_STRING_LENGTH]));
char* SHA512_Data __P((const u_int8_t*, size_t, char[SHA512_DIGEST_STRING_LENGTH]));

struct env_md_st *EVP_sha2_256 __P((void));
struct env_md_st *EVP_sha2_384 __P((void));
struct env_md_st *EVP_sha2_512 __P((void));

#ifdef HAVE_SHA2_IN_SHA_H
#define EVP_sha2_256 EVP_sha256
#define EVP_sha2_384 EVP_sha384
#define EVP_sha2_512 EVP_sha512
#endif

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif /* __SHA2_H__ */

