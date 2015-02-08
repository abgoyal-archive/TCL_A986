

#ifndef WMLIntrinsicEventHandler_h
#define WMLIntrinsicEventHandler_h

#if ENABLE(WML)
#include <wtf/HashMap.h>
#include <wtf/PassRefPtr.h>

#include "WMLIntrinsicEvent.h"

namespace WebCore {

enum WMLIntrinsicEventType {
    WMLIntrinsicEventUnknown = 0,
    WMLIntrinsicEventOnEnterForward,
    WMLIntrinsicEventOnEnterBackward,
    WMLIntrinsicEventOnTimer,
    WMLIntrinsicEventOnPick
};

class WMLIntrinsicEvent;

class WMLIntrinsicEventHandler {
public:
    WMLIntrinsicEventHandler();

    bool registerIntrinsicEvent(WMLIntrinsicEventType, PassRefPtr<WMLIntrinsicEvent>);
    void deregisterIntrinsicEvent(WMLIntrinsicEventType);
    void triggerIntrinsicEvent(WMLIntrinsicEventType) const;
    bool hasIntrinsicEvent(WMLIntrinsicEventType) const;

private:
    typedef HashMap<int, RefPtr<WMLIntrinsicEvent> > EventMap;
    EventMap m_events;
};

}

#endif
#endif
