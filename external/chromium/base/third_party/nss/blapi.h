/* $Id: blapi.h,v 1.27 2007/11/09 18:49:32 wtc%google.com Exp $ */

#ifndef _BLAPI_H_
#define _BLAPI_H_

#include "base/third_party/nss/blapit.h"

/******************************************/

extern SHA256Context *SHA256_NewContext(void);
extern void SHA256_DestroyContext(SHA256Context *cx, PRBool freeit);
extern void SHA256_Begin(SHA256Context *cx);
extern void SHA256_Update(SHA256Context *cx, const unsigned char *input,
			unsigned int inputLen);
extern void SHA256_End(SHA256Context *cx, unsigned char *digest,
		     unsigned int *digestLen, unsigned int maxDigestLen);
extern SECStatus SHA256_HashBuf(unsigned char *dest, const unsigned char *src,
			      unsigned int src_length);
extern SECStatus SHA256_Hash(unsigned char *dest, const char *src);
extern void SHA256_TraceState(SHA256Context *cx);
extern unsigned int SHA256_FlattenSize(SHA256Context *cx);
extern SECStatus SHA256_Flatten(SHA256Context *cx,unsigned char *space);
extern SHA256Context * SHA256_Resurrect(unsigned char *space, void *arg);
extern void SHA256_Clone(SHA256Context *dest, SHA256Context *src);

/******************************************/

extern SHA512Context *SHA512_NewContext(void);
extern void SHA512_DestroyContext(SHA512Context *cx, PRBool freeit);
extern void SHA512_Begin(SHA512Context *cx);
extern void SHA512_Update(SHA512Context *cx, const unsigned char *input,
			unsigned int inputLen);
extern void SHA512_End(SHA512Context *cx, unsigned char *digest,
		     unsigned int *digestLen, unsigned int maxDigestLen);
extern SECStatus SHA512_HashBuf(unsigned char *dest, const unsigned char *src,
			      unsigned int src_length);
extern SECStatus SHA512_Hash(unsigned char *dest, const char *src);
extern void SHA512_TraceState(SHA512Context *cx);
extern unsigned int SHA512_FlattenSize(SHA512Context *cx);
extern SECStatus SHA512_Flatten(SHA512Context *cx,unsigned char *space);
extern SHA512Context * SHA512_Resurrect(unsigned char *space, void *arg);
extern void SHA512_Clone(SHA512Context *dest, SHA512Context *src);

/******************************************/

extern SHA384Context *SHA384_NewContext(void);
extern void SHA384_DestroyContext(SHA384Context *cx, PRBool freeit);
extern void SHA384_Begin(SHA384Context *cx);
extern void SHA384_Update(SHA384Context *cx, const unsigned char *input,
			unsigned int inputLen);
extern void SHA384_End(SHA384Context *cx, unsigned char *digest,
		     unsigned int *digestLen, unsigned int maxDigestLen);
extern SECStatus SHA384_HashBuf(unsigned char *dest, const unsigned char *src,
			      unsigned int src_length);
extern SECStatus SHA384_Hash(unsigned char *dest, const char *src);
extern void SHA384_TraceState(SHA384Context *cx);
extern unsigned int SHA384_FlattenSize(SHA384Context *cx);
extern SECStatus SHA384_Flatten(SHA384Context *cx,unsigned char *space);
extern SHA384Context * SHA384_Resurrect(unsigned char *space, void *arg);
extern void SHA384_Clone(SHA384Context *dest, SHA384Context *src);

#endif /* _BLAPI_H_ */
