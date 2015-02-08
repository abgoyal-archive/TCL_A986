

#ifndef SHA256_H
#define SHA256_H

#define SHA256_MAC_LEN 32

void hmac_sha256_vector(const u8 *key, size_t key_len, size_t num_elem,
		      const u8 *addr[], const size_t *len, u8 *mac);
void hmac_sha256(const u8 *key, size_t key_len, const u8 *data,
		 size_t data_len, u8 *mac);
void sha256_prf(const u8 *key, size_t key_len, const char *label,
	      const u8 *data, size_t data_len, u8 *buf, size_t buf_len);

#endif /* SHA256_H */
