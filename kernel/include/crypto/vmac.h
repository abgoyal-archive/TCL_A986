

#ifndef __CRYPTO_VMAC_H
#define __CRYPTO_VMAC_H


#define VMAC_TAG_LEN	64
#define VMAC_KEY_SIZE	128/* Must be 128, 192 or 256			*/
#define VMAC_KEY_LEN	(VMAC_KEY_SIZE/8)
#define VMAC_NHBYTES	128/* Must 2^i for any 3 < i < 13 Standard = 128*/

struct vmac_ctx {
	u64 nhkey[(VMAC_NHBYTES/8)+2*(VMAC_TAG_LEN/64-1)];
	u64 polykey[2*VMAC_TAG_LEN/64];
	u64 l3key[2*VMAC_TAG_LEN/64];
	u64 polytmp[2*VMAC_TAG_LEN/64];
	u64 cached_nonce[2];
	u64 cached_aes[2];
	int first_block_processed;
};

typedef u64 vmac_t;

struct vmac_ctx_t {
	struct crypto_cipher *child;
	struct vmac_ctx __vmac_ctx;
};

#endif /* __CRYPTO_VMAC_H */
