

#ifndef ResourceResponse_h
#define ResourceResponse_h

#include "ResourceResponseBase.h"
#include <wtf/RetainPtr.h>

#ifdef __OBJC__
@class NSURLResponse;
#else
class NSURLResponse;
#endif

namespace WebCore {

class ResourceResponse : public ResourceResponseBase {
public:
    ResourceResponse()
        : m_isUpToDate(true)
    {
    }

    ResourceResponse(NSURLResponse* nsResponse)
        : m_nsResponse(nsResponse)
        , m_isUpToDate(false)
    {
        m_isNull = !nsResponse;
    }
    
    ResourceResponse(const KURL& url, const String& mimeType, long long expectedLength, const String& textEncodingName, const String& filename)
        : ResourceResponseBase(url, mimeType, expectedLength, textEncodingName, filename)
        , m_isUpToDate(true)
    {
    }

    unsigned memoryUsage() const
    {
        // FIXME: Find some programmatic lighweight way to calculate ResourceResponse and associated classes.
        // This is a rough estimate of resource overhead based on stats collected from the stress test.
        return 3072;
        /*  1280 * 2 +                // average size of ResourceResponse. Doubled to account for the WebCore copy and the CF copy.
                                      // Mostly due to the size of the hash maps, the Header Map strings and the URL.
            256 * 2                   // Overhead from ResourceRequest, doubled to account for WebCore copy and CF copy.
                                      // Mostly due to the URL and Header Map.
         */
    }

    NSURLResponse *nsURLResponse() const;

private:
    friend class ResourceResponseBase;

    void platformLazyInit();
    static bool platformCompare(const ResourceResponse& a, const ResourceResponse& b);

    RetainPtr<NSURLResponse> m_nsResponse;
    bool m_isUpToDate;
};

} // namespace WebCore

#endif // ResourceResponse_h
