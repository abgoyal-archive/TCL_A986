

#include "config.h"
#include "SSLKeyGenerator.h"

#include "PlatformBridge.h"

namespace WebCore {

void getSupportedKeySizes(Vector<String>& keys)
{
    keys = PlatformBridge::getSupportedKeyStrengthList();
}

String signedPublicKeyAndChallengeString(unsigned index, const String& challenge, const KURL& url)
{
    return PlatformBridge::getSignedPublicKeyAndChallengeString(index, challenge, url);
}

}
