

#include "includes.h"

#include "common.h"
#include "crypto/md5.h"
#include "crypto/crypto.h"
#include "chap.h"

void chap_md5(u8 id, const u8 *secret, size_t secret_len, const u8 *challenge,
	      size_t challenge_len, u8 *response)
{
	const u8 *addr[3];
	size_t len[3];

	addr[0] = &id;
	len[0] = 1;
	addr[1] = secret;
	len[1] = secret_len;
	addr[2] = challenge;
	len[2] = challenge_len;
	md5_vector(3, addr, len, response);
}
