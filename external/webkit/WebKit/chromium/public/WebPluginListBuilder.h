

#ifndef WebPluginListBuilder_h
#define WebPluginListBuilder_h

namespace WebKit {

class WebString;

// An interface for building a list of known plugins.
class WebPluginListBuilder {
public:
    virtual void addPlugin(
        const WebString& name, const WebString& description,
        const WebString& fileName) = 0;

    virtual void addMediaTypeToLastPlugin(
        const WebString& name, const WebString& description) = 0;

    virtual void addFileExtensionToLastMediaType(
        const WebString& fileExtension) = 0;
};

} // namespace WebKit

#endif
