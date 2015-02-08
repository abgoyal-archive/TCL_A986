

#ifndef WebURLError_h
#define WebURLError_h

#include "WebString.h"
#include "WebURL.h"

#if defined(WEBKIT_IMPLEMENTATION)
namespace WebCore { class ResourceError; }
#endif

namespace WebKit {

struct WebURLError {
    // A namespace for "reason" to support various layers generating
    // resource errors.  WebKit does not care about the value of this
    // string as it will just be passed via callbacks to the consumer.
    WebString domain;

    // A numeric error code detailing the reason for this error.  A value
    // of 0 means no error.  WebKit does not interpret the meaning of other
    // values and normally just forwards this error information back to the
    // embedder (see for example WebFrameClient).
    int reason;

    // The url that failed to load.
    WebURL unreachableURL;

    WebURLError() : reason(0) { }

#if defined(WEBKIT_IMPLEMENTATION)
    WebURLError(const WebCore::ResourceError&);
    WebURLError& operator=(const WebCore::ResourceError&);
    operator WebCore::ResourceError() const;
#endif
};

} // namespace WebKit

#endif
