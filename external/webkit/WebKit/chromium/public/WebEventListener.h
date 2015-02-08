

#ifndef WebEventListener_h
#define WebEventListener_h

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class Node; }
#endif

namespace WebKit {

class EventListenerWrapper;
class WebEvent;
class WebEventListenerPrivate;
class WebNode;
class WebString;

class WebEventListener {
public:
    WebEventListener();
    virtual ~WebEventListener();

    // Called when an event is received.
    virtual void handleEvent(const WebEvent&) = 0;

#if WEBKIT_IMPLEMENTATION
    void notifyEventListenerDeleted(EventListenerWrapper*);
    EventListenerWrapper* createEventListenerWrapper(const WebString& eventType, bool useCapture, WebCore::Node* node);
    EventListenerWrapper* getEventListenerWrapper(const WebString& eventType, bool useCapture, WebCore::Node* node);
#endif

private:
    WebEventListenerPrivate* m_private;
};

} // namespace WebKit

#endif
