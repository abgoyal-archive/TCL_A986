

#include "config.h"
#include "WebURLError.h"

#include "CString.h"
#include "KURL.h"
#include "ResourceError.h"

using namespace WebCore;

namespace WebKit {

WebURLError::WebURLError(const ResourceError& error)
{
    *this = error;
}

WebURLError& WebURLError::operator=(const ResourceError& error)
{
    if (error.isNull())
        *this = WebURLError();
    else {
        domain = error.domain();
        reason = error.errorCode();
        unreachableURL = KURL(ParsedURLString, error.failingURL());
    }
    return *this;
}

WebURLError::operator ResourceError() const
{
    if (!reason)
        return ResourceError();
    CString spec = unreachableURL.spec();
    return ResourceError(domain, reason,
                         String::fromUTF8(spec.data(), spec.length()),
                         String());
}

} // namespace WebKit
