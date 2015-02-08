

#ifndef WebSecurityPolicy_h
#define WebSecurityPolicy_h

#include "WebCommon.h"

namespace WebKit {

class WebString;
class WebURL;

class WebSecurityPolicy {
public:
    // Registers a URL scheme to be treated as a local scheme (i.e., with the
    // same security rules as those applied to "file" URLs).  This means that
    // normal pages cannot link to or access URLs of this scheme.
    WEBKIT_API static void registerURLSchemeAsLocal(const WebString&);

    // Registers a URL scheme to be treated as a noAccess scheme.  This means
    // that pages loaded with this URL scheme cannot access pages loaded with
    // any other URL scheme.
    WEBKIT_API static void registerURLSchemeAsNoAccess(const WebString&);

    // Support for whitelisting access to origins beyond the same-origin policy.
    WEBKIT_API static void whiteListAccessFromOrigin(
        const WebURL& sourceOrigin, const WebString& destinationProtocol,
        const WebString& destinationHost, bool allowDestinationSubdomains);
    WEBKIT_API static void resetOriginAccessWhiteLists();
    
    // Returns whether the url should be allowed to see the referrer
    // based on their respective protocols.
    WEBKIT_API static bool shouldHideReferrer(const WebURL& url, const WebString& referrer);

private:
    WebSecurityPolicy();
};

} // namespace WebKit

#endif
