

#ifndef WMLIntrinsicEvent_h
#define WMLIntrinsicEvent_h

#if ENABLE(WML)
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

#include "WMLTaskElement.h"

namespace WebCore {

class Document;

class WMLIntrinsicEvent : public RefCounted<WMLIntrinsicEvent> {
public:
    static PassRefPtr<WMLIntrinsicEvent> create(Document* document, const String& targetURL)
    {
        return adoptRef(new WMLIntrinsicEvent(document, targetURL));
    }

    static PassRefPtr<WMLIntrinsicEvent> createWithTask(WMLTaskElement* taskElement)
    {
        return adoptRef(new WMLIntrinsicEvent(taskElement));
    }

    WMLTaskElement* taskElement() const { return m_taskElement.get(); }

private:
    WMLIntrinsicEvent(Document*, const String& targetURL);
    WMLIntrinsicEvent(WMLTaskElement*);

    RefPtr<WMLTaskElement> m_taskElement;
};

}

#endif
#endif
