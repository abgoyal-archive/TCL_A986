

#ifndef WebPluginContainerImpl_h
#define WebPluginContainerImpl_h

// FIXME: This relative path is a temporary hack to support using this
// header from webkit/glue.
#include "../public/WebPluginContainer.h"

#include "Widget.h"
#include <wtf/PassRefPtr.h>
#include <wtf/Vector.h>

struct NPObject;

namespace WebCore {
class HTMLPlugInElement;
class IntRect;
class KeyboardEvent;
class MouseEvent;
class ResourceError;
class ResourceResponse;
}

namespace WebKit {

class WebPlugin;
class WebPluginLoadObserver;

class WebPluginContainerImpl : public WebCore::Widget, public WebPluginContainer {
public:
    static PassRefPtr<WebPluginContainerImpl> create(WebCore::HTMLPlugInElement* element, WebPlugin* webPlugin)
    {
        return adoptRef(new WebPluginContainerImpl(element, webPlugin));
    }

    // Widget methods
    virtual void setFrameRect(const WebCore::IntRect&);
    virtual void paint(WebCore::GraphicsContext*, const WebCore::IntRect&);
    virtual void invalidateRect(const WebCore::IntRect&);
    virtual void setFocus();
    virtual void show();
    virtual void hide();
    virtual void handleEvent(WebCore::Event*);
    virtual void frameRectsChanged();
    virtual void setParentVisible(bool);
    virtual void setParent(WebCore::ScrollView*);

    // WebPluginContainer methods
    virtual void invalidate();
    virtual void invalidateRect(const WebRect&);
    virtual void reportGeometry();
    virtual void clearScriptObjects();
    virtual NPObject* scriptableObjectForElement();
    virtual WebString executeScriptURL(const WebURL&, bool popupsAllowed);
    virtual void loadFrameRequest(const WebURLRequest&, const WebString& target, bool notifyNeeded, void* notifyData);

    // Resource load events for the plugin's source data:
    void didReceiveResponse(const WebCore::ResourceResponse&);
    void didReceiveData(const char *data, int dataLength);
    void didFinishLoading();
    void didFailLoading(const WebCore::ResourceError&);

    NPObject* scriptableObject();

    // This cannot be null.
    WebPlugin* plugin() { return m_webPlugin; }

    void willDestroyPluginLoadObserver(WebPluginLoadObserver*);

private:
    WebPluginContainerImpl(WebCore::HTMLPlugInElement* element, WebPlugin* webPlugin)
        : m_element(element)
        , m_webPlugin(webPlugin) { }
    ~WebPluginContainerImpl();

    void handleMouseEvent(WebCore::MouseEvent*);
    void handleKeyboardEvent(WebCore::KeyboardEvent*);

    void calculateGeometry(const WebCore::IntRect& frameRect,
                           WebCore::IntRect& windowRect,
                           WebCore::IntRect& clipRect,
                           Vector<WebCore::IntRect>& cutOutRects);
    WebCore::IntRect windowClipRect() const;
    void windowCutOutRects(const WebCore::IntRect& frameRect,
                           Vector<WebCore::IntRect>& cutOutRects);

    WebCore::HTMLPlugInElement* m_element;
    WebPlugin* m_webPlugin;
    Vector<WebPluginLoadObserver*> m_pluginLoadObservers;
};

} // namespace WebKit

#endif
