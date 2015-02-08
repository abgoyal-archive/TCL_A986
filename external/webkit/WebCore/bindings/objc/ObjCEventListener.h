

#ifndef ObjCEventListener_h
#define ObjCEventListener_h

#include "EventListener.h"

#include <wtf/PassRefPtr.h>

@protocol DOMEventListener;

namespace WebCore {

    class ObjCEventListener : public EventListener {
    public:
        static PassRefPtr<ObjCEventListener> wrap(id <DOMEventListener>);

        static const ObjCEventListener* cast(const EventListener* listener)
        {
            return listener->type() == ObjCEventListenerType
                ? static_cast<const ObjCEventListener*>(listener)
                : 0;
        }

        virtual bool operator==(const EventListener& other);

    private:
        static ObjCEventListener* find(id <DOMEventListener>);

        ObjCEventListener(id <DOMEventListener>);
        virtual ~ObjCEventListener();

        virtual void handleEvent(ScriptExecutionContext*, Event*);

        id <DOMEventListener> m_listener;
    };

} // namespace WebCore

#endif
