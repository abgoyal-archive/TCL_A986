

#ifndef WebPlugin_h
#define WebPlugin_h

#include "WebCanvas.h"

struct NPObject;

namespace WebKit {

class WebDataSource;
class WebFrame;
class WebInputEvent;
class WebPluginContainer;
class WebURL;
class WebURLResponse;
struct WebCursorInfo;
struct WebPluginParams;
struct WebRect;
struct WebURLError;
template <typename T> class WebVector;

class WebPlugin {
public:
    virtual bool initialize(WebPluginContainer*) = 0;
    virtual void destroy() = 0;

    virtual NPObject* scriptableObject() = 0;

    virtual void paint(WebCanvas*, const WebRect&) = 0;

    // Coordinates are relative to the containing window.
    virtual void updateGeometry(
        const WebRect& frameRect, const WebRect& clipRect,
        const WebVector<WebRect>& cutOutsRects, bool isVisible) = 0;

    virtual void updateFocus(bool) = 0;
    virtual void updateVisibility(bool) = 0;

    virtual bool acceptsInputEvents() = 0;
    virtual bool handleInputEvent(const WebInputEvent&, WebCursorInfo&) = 0;

    virtual void didReceiveResponse(const WebURLResponse&) = 0;
    virtual void didReceiveData(const char* data, int dataLength) = 0;
    virtual void didFinishLoading() = 0;
    virtual void didFailLoading(const WebURLError&) = 0;

    // Called in response to WebPluginContainer::loadFrameRequest
    virtual void didFinishLoadingFrameRequest(
        const WebURL&, void* notifyData) = 0;
    virtual void didFailLoadingFrameRequest(
        const WebURL&, void* notifyData, const WebURLError&) = 0;

protected:
    ~WebPlugin() { }
};

} // namespace WebKit

#endif
