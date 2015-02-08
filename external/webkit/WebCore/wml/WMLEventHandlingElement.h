

#ifndef WMLEventHandlingElement_h
#define WMLEventHandlingElement_h

#if ENABLE(WML)
#include "WMLIntrinsicEventHandler.h"

#include <wtf/OwnPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class WMLElement;
class WMLDoElement;

class WMLEventHandlingElement {
public:
    WMLEventHandlingElement();
    ~WMLEventHandlingElement();

    WMLIntrinsicEventHandler* eventHandler() const { return m_eventHandler.get(); }
    void createEventHandlerIfNeeded();

    Vector<WMLDoElement*>& doElements() { return m_doElements; }
    void registerDoElement(WMLDoElement*, Document*);
    void deregisterDoElement(WMLDoElement*);

private:
    OwnPtr<WMLIntrinsicEventHandler> m_eventHandler;
    Vector<WMLDoElement*> m_doElements;
};

WMLEventHandlingElement* toWMLEventHandlingElement(WMLElement*);

}

#endif
#endif
