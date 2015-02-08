

#ifndef WebPluginListBuilderImpl_h
#define WebPluginListBuilderImpl_h

#include "WebPluginListBuilder.h"

#include "PluginData.h"
#include <wtf/Vector.h>

namespace WebKit {

class WebPluginListBuilderImpl : public WebPluginListBuilder {
public:
    WebPluginListBuilderImpl(Vector<WebCore::PluginInfo*>* results) : m_results(results) { }

    // WebPluginListBuilder methods:
    virtual void addPlugin(const WebString& name, const WebString& description, const WebString& fileName);
    virtual void addMediaTypeToLastPlugin(const WebString& name, const WebString& description);
    virtual void addFileExtensionToLastMediaType(const WebString& extension);

private:
    Vector<WebCore::PluginInfo*>* m_results;
};

} // namespace WebKit

#endif
