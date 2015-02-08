

#ifndef ResourceResponse_h
#define ResourceResponse_h

#include "ResourceResponseBase.h"

namespace WebCore {

class ResourceResponse : public ResourceResponseBase {
public:
    ResourceResponse()
    {
    }

    ResourceResponse(const KURL& url, const String& mimeType, long long expectedLength, const String& textEncodingName, const String& filename)
        : ResourceResponseBase(url, mimeType, expectedLength, textEncodingName, filename)
    {
    }
};

} // namespace WebCore

#endif // ResourceResponse_h
