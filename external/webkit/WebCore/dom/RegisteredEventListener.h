

#ifndef RegisteredEventListener_h
#define RegisteredEventListener_h

#include "AtomicString.h"
#include "EventListener.h"

namespace WebCore {

    class RegisteredEventListener {
    public:
        RegisteredEventListener(PassRefPtr<EventListener> listener, bool useCapture)
            : listener(listener)
            , useCapture(useCapture)
        {
        }

        RefPtr<EventListener> listener;
        bool useCapture;
    };
    
    inline bool operator==(const RegisteredEventListener& a, const RegisteredEventListener& b)
    {
        return *a.listener == *b.listener && a.useCapture == b.useCapture;
    }

} // namespace WebCore

#endif // RegisteredEventListener_h
