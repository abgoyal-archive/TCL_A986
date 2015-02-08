

#ifndef SSLKeyGenerator_h
#define SSLKeyGenerator_h

#include <wtf/Vector.h>
#include "PlatformString.h"

namespace WebCore {

    class KURL;

    // Returns strings representing key sizes that may be used
    // for the <keygen> tag. The first string is displayed as the default
    // key size in the <keygen> menu.
    void getSupportedKeySizes(Vector<String>& sizes);

    // This function handles the <keygen> tag in form elements.
    // Returns a signed copy of the combined challenge string and public
    // key (from a newly generated key pair).
    String signedPublicKeyAndChallengeString(unsigned keySizeIndex, const String& challengeString, const KURL&);

} // namespace WebCore

#endif // SSLKeyGenerator_h
