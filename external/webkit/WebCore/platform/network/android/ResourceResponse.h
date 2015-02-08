

#ifndef ResourceResponse_h
#define ResourceResponse_h

#include "NotImplemented.h"
#include "ResourceResponseBase.h"

namespace WebCore {

class ResourceResponse : public ResourceResponseBase {
public:
    ResourceResponse() : ResourceResponseBase() { }

    ResourceResponse(const KURL& url, const String& mimeType, long long expectedLength, const String& textEncodingName, const String& filename)
        : ResourceResponseBase(url, mimeType, expectedLength, textEncodingName, filename) { }

private:
    friend class ResourceResponseBase;

    void doUpdateResourceResponse()
    {
        notImplemented();
    }
};

} // namespace WebCore

#endif // ResourceResponse_h
