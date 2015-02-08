

#ifndef WebPluginContainer_h
#define WebPluginContainer_h

struct NPObject;

namespace WebKit {

class WebString;
class WebURL;
class WebURLRequest;
struct WebRect;

class WebPluginContainer {
public:
    virtual void invalidate() = 0;
    virtual void invalidateRect(const WebRect&) = 0;

    // Causes the container to report its current geometry via
    // WebPlugin::updateGeometry.
    virtual void reportGeometry() = 0;

    // Drop any references to script objects allocated by the plugin.
    // These are objects derived from WebPlugin::scriptableObject.  This is
    // called when the plugin is being destroyed or if it needs to be
    // re-initialized.
    virtual void clearScriptObjects() = 0;

    // Returns the scriptable object associated with the DOM element
    // containing the plugin.
    virtual NPObject* scriptableObjectForElement() = 0;

    // Executes a "javascript:" URL on behalf of the plugin in the context
    // of the frame containing the plugin.  Returns the result of script
    // execution, if any.
    virtual WebString executeScriptURL(const WebURL&, bool popupsAllowed) = 0;

    // Loads an URL in the specified frame (or the frame containing this
    // plugin if target is empty).  If notifyNeeded is true, then upon
    // completion, WebPlugin::didFinishLoadingFrameRequest is called if the
    // load was successful or WebPlugin::didFailLoadingFrameRequest is
    // called if the load failed.  The given notifyData is passed along to
    // the callback.
    virtual void loadFrameRequest(
        const WebURLRequest&, const WebString& target, bool notifyNeeded, void* notifyData) = 0;

protected:
    ~WebPluginContainer() { }
};

} // namespace WebKit

#endif
