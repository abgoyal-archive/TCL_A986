// This is mozilla/security/manager/ssl/src/md4.h, CVS rev. 1.1, with trivial
// changes to port it to our source tree.
//
// WARNING: MD4 is cryptographically weak.  Do not use MD4 except in NTLM
// authentication.

/* vim:set ts=2 sw=2 et cindent: */

#ifndef NET_HTTP_MD4_H_
#define NET_HTTP_MD4_H_

#include "base/basictypes.h"

namespace net {
namespace weak_crypto {

void MD4Sum(const uint8 *input, uint32 inputLen, uint8 *result);

}  // namespace net::weak_crypto
}  // namespace net

#endif  // NET_HTTP_MD4_H_
