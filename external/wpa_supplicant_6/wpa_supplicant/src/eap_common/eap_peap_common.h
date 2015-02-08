

#ifndef EAP_PEAP_COMMON_H
#define EAP_PEAP_COMMON_H

void peap_prfplus(int version, const u8 *key, size_t key_len,
		  const char *label, const u8 *seed, size_t seed_len,
		  u8 *buf, size_t buf_len);

#endif /* EAP_PEAP_COMMON_H */
