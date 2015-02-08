

#ifndef WebPluginParams_h
#define WebPluginParams_h

#include "WebString.h"
#include "WebURL.h"
#include "WebVector.h"

namespace WebKit {

struct WebPluginParams {
    WebURL url;
    WebString mimeType;
    WebVector<WebString> attributeNames;
    WebVector<WebString> attributeValues;
    bool loadManually;
};

} // namespace WebKit

#endif
