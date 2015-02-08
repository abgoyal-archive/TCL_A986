
 
#ifndef BeforeTextInsertedEvent_h
#define BeforeTextInsertedEvent_h

#include "Event.h"

namespace WebCore {

class BeforeTextInsertedEvent : public Event {
public:
    static PassRefPtr<BeforeTextInsertedEvent> create(const String& text)
    {
        return adoptRef(new BeforeTextInsertedEvent(text));
    }

    virtual bool isBeforeTextInsertedEvent() const { return true; }
  
    const String& text() const { return m_text; }
    void setText(const String& s) { m_text = s; }

private:
    BeforeTextInsertedEvent(const String&);

    String m_text;
};

} // namespace

#endif
