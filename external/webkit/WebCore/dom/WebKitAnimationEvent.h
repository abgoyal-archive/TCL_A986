

#ifndef WebKitAnimationEvent_h
#define WebKitAnimationEvent_h

#include "Event.h"

namespace WebCore {
        
    class WebKitAnimationEvent : public Event {
    public:
        static PassRefPtr<WebKitAnimationEvent> create()
        {
            return adoptRef(new WebKitAnimationEvent);
        }
        static PassRefPtr<WebKitAnimationEvent> create(const AtomicString& type, const String& animationName, double elapsedTime)
        {
            return adoptRef(new WebKitAnimationEvent(type, animationName, elapsedTime));
        }

        virtual ~WebKitAnimationEvent();
        
        void initWebKitAnimationEvent(const AtomicString& type, 
                                bool canBubbleArg,
                                bool cancelableArg,
                                const String& animationName,
                                double elapsedTime);
                        
        const String& animationName() const;
        double elapsedTime() const;
        
        virtual bool isWebKitAnimationEvent() const { return true; }
        
    private:
        WebKitAnimationEvent();
        WebKitAnimationEvent(const AtomicString& type, const String& animationName, double elapsedTime);

        String m_animationName;
        double m_elapsedTime;
    };
    
} // namespace WebCore

#endif // WebKitAnimationEvent_h
