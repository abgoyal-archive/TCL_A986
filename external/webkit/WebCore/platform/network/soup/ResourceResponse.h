

#ifndef ResourceResponse_h
#define ResourceResponse_h

#include "ResourceResponseBase.h"

#include <libsoup/soup.h>

namespace WebCore {

class ResourceResponse : public ResourceResponseBase {
public:
    ResourceResponse()
        : ResourceResponseBase()
    {
    }

    ResourceResponse(const KURL& url, const String& mimeType, long long expectedLength, const String& textEncodingName, const String& filename)
        : ResourceResponseBase(url, mimeType, expectedLength, textEncodingName, filename)
    {
    }

    ResourceResponse(SoupMessage* soupMessage)
        : ResourceResponseBase()
    {
        updateFromSoupMessage(soupMessage);
    }

    SoupMessage* toSoupMessage() const;
    void updateFromSoupMessage(SoupMessage* soupMessage);

private:
    friend class ResourceResponseBase;

    void doUpdateResourceResponse()
    {
    }
};

} // namespace WebCore

#endif // ResourceResponse_h
