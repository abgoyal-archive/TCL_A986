

#ifndef WebMimeRegistry_h
#define WebMimeRegistry_h

#include "WebCommon.h"

namespace WebKit {

class WebString;

class WebMimeRegistry {
public:
    enum SupportsType { IsNotSupported, IsSupported, MayBeSupported };

    virtual SupportsType supportsMIMEType(const WebString& mimeType) = 0;
    virtual SupportsType supportsImageMIMEType(const WebString& mimeType) = 0;
    virtual SupportsType supportsJavaScriptMIMEType(const WebString& mimeType) = 0;
    virtual SupportsType supportsMediaMIMEType(const WebString& mimeType,
                                               const WebString& codecs) = 0;
    virtual SupportsType supportsNonImageMIMEType(const WebString& mimeType) = 0;

    virtual WebString mimeTypeForExtension(const WebString& fileExtension) = 0;
    virtual WebString mimeTypeFromFile(const WebString& filePath) = 0;
    virtual WebString preferredExtensionForMIMEType(const WebString& mimeType) = 0;

protected:
    ~WebMimeRegistry() { }
};

} // namespace WebKit

#endif
