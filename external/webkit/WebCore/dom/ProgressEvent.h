

#ifndef ProgressEvent_h
#define ProgressEvent_h

#include "Event.h"

namespace WebCore {
    
    class ProgressEvent : public Event {
    public:
        static PassRefPtr<ProgressEvent> create()
        {
            return adoptRef(new ProgressEvent);
        }
        static PassRefPtr<ProgressEvent> create(const AtomicString& type, bool lengthComputable, unsigned loaded, unsigned total)
        {
            return adoptRef(new ProgressEvent(type, lengthComputable, loaded, total));
        }

        void initProgressEvent(const AtomicString& typeArg, 
                               bool canBubbleArg,
                               bool cancelableArg,
                               bool lengthComputableArg,
                               unsigned loadedArg,
                               unsigned totalArg);
        
        bool lengthComputable() const { return m_lengthComputable; }
        unsigned loaded() const { return m_loaded; }
        unsigned total() const { return m_total; }
        
        virtual bool isProgressEvent() const { return true; }
        
    protected:
        ProgressEvent();
        ProgressEvent(const AtomicString& type, bool lengthComputable, unsigned loaded, unsigned total);

    private:
        bool m_lengthComputable;
        unsigned m_loaded;
        unsigned m_total;
    };
}

#endif // ProgressEvent_h

