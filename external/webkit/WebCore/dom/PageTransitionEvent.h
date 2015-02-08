

#ifndef PageTransitionEvent_h
#define PageTransitionEvent_h

#include "Event.h"

namespace WebCore {

    class PageTransitionEvent : public Event {
    public:
        static PassRefPtr<PageTransitionEvent> create()
        {
            return adoptRef(new PageTransitionEvent);
        }
        static PassRefPtr<PageTransitionEvent> create(const AtomicString& type, bool persisted)
        {
            return adoptRef(new PageTransitionEvent(type, persisted));
        }

        virtual ~PageTransitionEvent();

        void initPageTransitionEvent(const AtomicString& type, 
                                bool canBubbleArg,
                                bool cancelableArg,
                                bool persisted);

        virtual bool isPageTransitionEvent() const { return true; }

        bool persisted() const { return m_persisted; }

    private:
        PageTransitionEvent();
        PageTransitionEvent(const AtomicString& type, bool persisted);

        bool m_persisted;
    };

} // namespace WebCore

#endif // PageTransitionEvent_h
