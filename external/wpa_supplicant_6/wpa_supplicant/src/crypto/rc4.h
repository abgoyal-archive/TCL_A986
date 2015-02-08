

#ifndef RC4_H
#define RC4_H

void rc4_skip(const u8 *key, size_t keylen, size_t skip,
	      u8 *data, size_t data_len);

#endif /* RC4_H */
