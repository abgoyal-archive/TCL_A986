

#ifndef EventListenerWrapper_h
#define EventListenerWrapper_h

#include "EventListener.h"

namespace WebCore {
class ScriptExecutionContext;
}

using namespace WebCore;

namespace WebKit {

class WebEventListener;

class EventListenerWrapper : public EventListener {
public:
    EventListenerWrapper(WebEventListener*);
    ~EventListenerWrapper();

    virtual bool operator==(const EventListener&);
    virtual void handleEvent(ScriptExecutionContext*, Event*);

    void webEventListenerDeleted();

private:
    WebEventListener* m_webEventListener;
};

} // namespace WebKit

#endif
