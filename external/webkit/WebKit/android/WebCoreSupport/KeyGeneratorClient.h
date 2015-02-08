

#ifndef KEY_GENERATOR_CLIENT_H
#define KEY_GENERATOR_CLIENT_H

#include "KURL.h"
#include "PlatformString.h"

#include <wtf/Vector.h>

using namespace WebCore;

namespace android {

class KeyGeneratorClient {
public:
    virtual ~KeyGeneratorClient() { }
    virtual WTF::Vector<String> getSupportedKeyStrengthList() = 0;
    virtual String getSignedPublicKeyAndChallengeString(unsigned index,
            const String& challenge, const KURL& url) = 0;
    };
}
#endif
