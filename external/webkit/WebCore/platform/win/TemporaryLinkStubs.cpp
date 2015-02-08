

#include "config.h"

#include "NotImplemented.h"
#include "SSLKeyGenerator.h"

namespace WebCore {

// <keygen>
String signedPublicKeyAndChallengeString(unsigned, const String&, const KURL&) { notImplemented(); return String(); }
void getSupportedKeySizes(Vector<String>&) { notImplemented(); }
#if PLATFORM(CAIRO)
void populateFontDatabase() { /* Not needed for GDI fonts */ }
#endif
} // namespace WebCore
