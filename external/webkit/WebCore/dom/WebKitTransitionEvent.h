

#ifndef WebKitTransitionEvent_h
#define WebKitTransitionEvent_h

#include "Event.h"

namespace WebCore {
        
    class WebKitTransitionEvent : public Event {
    public:
        static PassRefPtr<WebKitTransitionEvent> create()
        {
            return adoptRef(new WebKitTransitionEvent);
        }
        static PassRefPtr<WebKitTransitionEvent> create(const AtomicString& type, const String& animationName, double elapsedTime)
        {
            return adoptRef(new WebKitTransitionEvent(type, animationName, elapsedTime));
        }

        virtual ~WebKitTransitionEvent();
        
        void initWebKitTransitionEvent(const AtomicString& type, 
                                bool canBubbleArg,
                                bool cancelableArg,
                                const String& propertyName,
                                double elapsedTime);
                        
        const String& propertyName() const;
        double elapsedTime() const;
        
        virtual bool isWebKitTransitionEvent() const { return true; }
        
    private:
        WebKitTransitionEvent();
        WebKitTransitionEvent(const AtomicString& type, const String& propertyName, double elapsedTime);

        String m_propertyName;
        double m_elapsedTime;
    };
    
} // namespace WebCore

#endif // WebKitTransitionEvent_h
