

#include "config.h"

#include "AXObjectCache.h"
#include "DNS.h"
#include "Editor.h"
#include "FrameView.h"
#include "FTPDirectoryDocument.h"
#include "NotImplemented.h"
#include "PluginView.h"
#include <float.h>

using namespace WebCore;

/********************************************************/
/* Completely empty stubs (mostly to allow DRT to run): */
/********************************************************/

namespace WebCore {
void getSupportedKeySizes(Vector<String>&) { notImplemented(); }
String signedPublicKeyAndChallengeString(unsigned keySizeIndex, const String &challengeString, const KURL &url) { return String(); }
float userIdleTime() { notImplemented(); return FLT_MAX; } // return an arbitrarily high userIdleTime so that releasing pages from the page cache isn't postponed

}
