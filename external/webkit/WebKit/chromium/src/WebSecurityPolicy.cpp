

#include "config.h"
#include "WebSecurityPolicy.h"

#include "FrameLoader.h"
#include "SecurityOrigin.h"

#include "WebString.h"
#include "WebURL.h"

using namespace WebCore;

namespace WebKit {

void WebSecurityPolicy::registerURLSchemeAsLocal(const WebString& scheme)
{
    SecurityOrigin::registerURLSchemeAsLocal(scheme);
}

void WebSecurityPolicy::registerURLSchemeAsNoAccess(const WebString& scheme)
{
    SecurityOrigin::registerURLSchemeAsNoAccess(scheme);
}

void WebSecurityPolicy::whiteListAccessFromOrigin(const WebURL& sourceOrigin,
    const WebString& destinationProtocol,
    const WebString& destinationHost,
    bool allowDestinationSubdomains)
{
    SecurityOrigin::whiteListAccessFromOrigin(
        *SecurityOrigin::create(sourceOrigin), destinationProtocol,
        destinationHost, allowDestinationSubdomains);
}

void WebSecurityPolicy::resetOriginAccessWhiteLists()
{
    SecurityOrigin::resetOriginAccessWhiteLists();
}

bool WebSecurityPolicy::shouldHideReferrer(const WebURL& url, const WebString& referrer)
{
    return SecurityOrigin::shouldHideReferrer(url, referrer);
}

} // namespace WebKit
