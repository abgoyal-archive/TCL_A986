

#ifndef WebEventListenerPrivate_h
#define WebEventListenerPrivate_h

#include "WebString.h"

#include <wtf/Vector.h>

namespace WebCore {
class Node;
}

using namespace WebCore;

namespace WebKit {

class EventListenerWrapper;
class WebEventListener;

class WebEventListenerPrivate {
public:
    WebEventListenerPrivate(WebEventListener* webEventListener);
    ~WebEventListenerPrivate();

    EventListenerWrapper* createEventListenerWrapper(
        const WebString& eventType, bool useCapture, Node* node);

    // Gets the ListenerEventWrapper for a specific node.
    // Used by WebNode::removeEventListener().
    EventListenerWrapper* getEventListenerWrapper(
        const WebString& eventType, bool useCapture, Node* node);

    // Called by the WebEventListener when it is about to be deleted.
    void webEventListenerDeleted();

    // Called by the EventListenerWrapper when it is about to be deleted.
    void eventListenerDeleted(EventListenerWrapper* eventListener);

    struct ListenerInfo {
        ListenerInfo(const WebString& eventType, bool useCapture,
                     EventListenerWrapper* eventListenerWrapper,
                     Node* node)
            : eventType(eventType)
            , useCapture(useCapture)
            , eventListenerWrapper(eventListenerWrapper)
            , node(node)
        {
        }

        WebString eventType;
        bool useCapture;
        EventListenerWrapper* eventListenerWrapper;
        Node* node;
    };

private:
    WebEventListener* m_webEventListener;

    // We keep a list of the wrapper for the WebKit EventListener, it is needed
    // to implement WebNode::removeEventListener().
    Vector<ListenerInfo> m_listenerWrappers;
};

} // namespace WebKit

#endif
