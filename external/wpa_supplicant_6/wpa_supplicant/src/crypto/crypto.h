

#ifndef CRYPTO_H
#define CRYPTO_H

void md4_vector(size_t num_elem, const u8 *addr[], const size_t *len, u8 *mac);

void md5_vector(size_t num_elem, const u8 *addr[], const size_t *len, u8 *mac);

void sha1_vector(size_t num_elem, const u8 *addr[], const size_t *len,
		 u8 *mac);

int __must_check fips186_2_prf(const u8 *seed, size_t seed_len, u8 *x,
			       size_t xlen);

void sha256_vector(size_t num_elem, const u8 *addr[], const size_t *len,
		   u8 *mac);

void des_encrypt(const u8 *clear, const u8 *key, u8 *cypher);

void * aes_encrypt_init(const u8 *key, size_t len);

void aes_encrypt(void *ctx, const u8 *plain, u8 *crypt);

void aes_encrypt_deinit(void *ctx);

void * aes_decrypt_init(const u8 *key, size_t len);

void aes_decrypt(void *ctx, const u8 *crypt, u8 *plain);

void aes_decrypt_deinit(void *ctx);


enum crypto_hash_alg {
	CRYPTO_HASH_ALG_MD5, CRYPTO_HASH_ALG_SHA1,
	CRYPTO_HASH_ALG_HMAC_MD5, CRYPTO_HASH_ALG_HMAC_SHA1
};

struct crypto_hash;

struct crypto_hash * crypto_hash_init(enum crypto_hash_alg alg, const u8 *key,
				      size_t key_len);

void crypto_hash_update(struct crypto_hash *ctx, const u8 *data, size_t len);

int crypto_hash_finish(struct crypto_hash *ctx, u8 *hash, size_t *len);


enum crypto_cipher_alg {
	CRYPTO_CIPHER_NULL = 0, CRYPTO_CIPHER_ALG_AES, CRYPTO_CIPHER_ALG_3DES,
	CRYPTO_CIPHER_ALG_DES, CRYPTO_CIPHER_ALG_RC2, CRYPTO_CIPHER_ALG_RC4
};

struct crypto_cipher;

struct crypto_cipher * crypto_cipher_init(enum crypto_cipher_alg alg,
					  const u8 *iv, const u8 *key,
					  size_t key_len);

int __must_check crypto_cipher_encrypt(struct crypto_cipher *ctx,
				       const u8 *plain, u8 *crypt, size_t len);

int __must_check crypto_cipher_decrypt(struct crypto_cipher *ctx,
				       const u8 *crypt, u8 *plain, size_t len);

void crypto_cipher_deinit(struct crypto_cipher *ctx);


struct crypto_public_key;
struct crypto_private_key;

struct crypto_public_key * crypto_public_key_import(const u8 *key, size_t len);

struct crypto_private_key * crypto_private_key_import(const u8 *key,
						      size_t len);

struct crypto_public_key * crypto_public_key_from_cert(const u8 *buf,
						       size_t len);

int __must_check crypto_public_key_encrypt_pkcs1_v15(
	struct crypto_public_key *key, const u8 *in, size_t inlen,
	u8 *out, size_t *outlen);

int __must_check crypto_private_key_decrypt_pkcs1_v15(
	struct crypto_private_key *key, const u8 *in, size_t inlen,
	u8 *out, size_t *outlen);

int __must_check crypto_private_key_sign_pkcs1(struct crypto_private_key *key,
					       const u8 *in, size_t inlen,
					       u8 *out, size_t *outlen);

void crypto_public_key_free(struct crypto_public_key *key);

void crypto_private_key_free(struct crypto_private_key *key);

int __must_check crypto_public_key_decrypt_pkcs1(
	struct crypto_public_key *key, const u8 *crypt, size_t crypt_len,
	u8 *plain, size_t *plain_len);

int __must_check crypto_global_init(void);

void crypto_global_deinit(void);

int __must_check crypto_mod_exp(const u8 *base, size_t base_len,
				const u8 *power, size_t power_len,
				const u8 *modulus, size_t modulus_len,
				u8 *result, size_t *result_len);

#endif /* CRYPTO_H */
