
#ifndef _BIGNUM_H_
#define _BIGNUM_H_

#include "includes.h"

void m_mp_init(mp_int *mp);
void m_mp_init_multi(mp_int *mp, ...);
void bytes_to_mp(mp_int *mp, const unsigned char* bytes, unsigned int len);
void sha1_process_mp(hash_state *hs, mp_int *mp);

#endif /* _BIGNUM_H_ */
