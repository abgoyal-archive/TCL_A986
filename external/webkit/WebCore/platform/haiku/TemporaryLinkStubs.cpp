

#include "config.h"

#include "KURL.h"
#include "NotImplemented.h"
#include "PlatformString.h"
#include "SSLKeyGenerator.h"
#include "SystemTime.h"

using namespace WebCore;

Vector<char> loadResourceIntoArray(const char*)
{
    notImplemented();
    return Vector<char>();
}

namespace WebCore {

String signedPublicKeyAndChallengeString(unsigned keySizeIndex, const String &challengeString, const KURL &url)
{
    return String();
}

void getSupportedKeySizes(Vector<String>&)
{
    notImplemented();
}

float userIdleTime()
{
    notImplemented();
    return 0;
}

void callOnMainThread(void (*)())
{
    notImplemented();
}

String KURL::fileSystemPath() const
{
    notImplemented();
    return String();
}

} // namespace WebCore

