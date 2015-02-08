

#ifndef BeforeLoadEvent_h
#define BeforeLoadEvent_h

#include "Event.h"
#include "EventNames.h"

namespace WebCore {

class BeforeLoadEvent : public Event {
public:
    virtual bool isBeforeLoadEvent() const { return true; }

    static PassRefPtr<BeforeLoadEvent> create(const String& url)
    {
        return adoptRef(new BeforeLoadEvent(url));
    }

    void initBeforeLoadEvent(const AtomicString& type, bool canBubble, bool cancelable, const String& url)
    {
        if (dispatched())
            return;
        
        initEvent(type, canBubble, cancelable);

        m_url = url;
    }

    const String& url() const { return m_url; }

private:
    BeforeLoadEvent(const String& url)
        : Event(eventNames().beforeloadEvent, false, true)
        , m_url(url)
    {}

    String m_url;
};

} // namespace WebCore

#endif // BeforeLoadEvent_h
