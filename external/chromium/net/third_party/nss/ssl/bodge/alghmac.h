
#ifndef _ALGHMAC_H_
#define _ALGHMAC_H_

typedef struct HMACContextStr HMACContext;

SEC_BEGIN_PROTOS

/* destroy HMAC context */
extern void
HMAC_Destroy(HMACContext *cx, PRBool freeit);

extern HMACContext *
HMAC_Create(const SECHashObject *hash_obj, const unsigned char *secret, 
	    unsigned int secret_len, PRBool isFIPS);

/* like HMAC_Create, except caller allocates HMACContext. */
SECStatus
HMAC_Init(HMACContext *cx, const SECHashObject *hash_obj, 
	  const unsigned char *secret, unsigned int secret_len, PRBool isFIPS);

/* reset HMAC for a fresh round */
extern void
HMAC_Begin(HMACContext *cx);

extern void 
HMAC_Update(HMACContext *cx, const unsigned char *data, unsigned int data_len);

extern SECStatus
HMAC_Finish(HMACContext *cx, unsigned char *result, unsigned int *result_len,
	    unsigned int max_result_len);

extern HMACContext *
HMAC_Clone(HMACContext *cx);

SEC_END_PROTOS

#endif
