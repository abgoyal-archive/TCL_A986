

#include "config.h"
#include "SSLKeyGenerator.h"

#include "ChromiumBridge.h"
#include "PlatformString.h"

namespace WebCore {

// These are defined in webkit/api/src/LocalizedStrings.cpp.
String keygenMenuHighGradeKeySize();
String keygenMenuMediumGradeKeySize();

void getSupportedKeySizes(Vector<String>& sizes)
{
    sizes.resize(2);
    sizes[0] = keygenMenuHighGradeKeySize();
    sizes[1] = keygenMenuMediumGradeKeySize();
}

String signedPublicKeyAndChallengeString(unsigned keySizeIndex,
                                         const String& challengeString,
                                         const KURL& url)
{
    return ChromiumBridge::signedPublicKeyAndChallengeString(keySizeIndex,
                                                             challengeString,
                                                             url);
}

} // namespace WebCore
