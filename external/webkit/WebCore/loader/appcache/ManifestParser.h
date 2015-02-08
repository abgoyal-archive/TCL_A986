

#ifndef ManifestParser_h
#define ManifestParser_h

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

#include "ApplicationCache.h"

namespace WebCore {

    class KURL;

    struct Manifest {
        Vector<KURL> onlineWhitelistedURLs;
        HashSet<String> explicitURLs;
        FallbackURLVector fallbackURLs;
        bool allowAllNetworkRequests; // Wildcard found in NETWORK section.
    };

    bool parseManifest(const KURL& manifestURL, const char* data, int length, Manifest&);

}

#endif // ENABLE(OFFLINE_WEB_APPLICATIONS)

#endif // ManifestParser_h
