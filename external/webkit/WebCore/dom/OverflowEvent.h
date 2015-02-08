

#ifndef OverflowEvent_h
#define OverflowEvent_h

#include "Event.h"

namespace WebCore {
    
    class OverflowEvent : public Event {
    public:
        enum orientType {
            VERTICAL   = 0,
            HORIZONTAL = 1,
            BOTH       = 2
        };

        static PassRefPtr<OverflowEvent> create()
        {
            return adoptRef(new OverflowEvent);
        }
        static PassRefPtr<OverflowEvent> create(bool horizontalOverflowChanged, bool horizontalOverflow, bool verticalOverflowChanged, bool verticalOverflow)
        {
            return adoptRef(new OverflowEvent(horizontalOverflowChanged, horizontalOverflow, verticalOverflowChanged, verticalOverflow));
        }

        void initOverflowEvent(unsigned short orient, bool horizontalOverflow, bool verticalOverflow);

        unsigned short orient() const { return m_orient; }
        bool horizontalOverflow() const { return m_horizontalOverflow; }
        bool verticalOverflow() const { return m_verticalOverflow; }

        virtual bool isOverflowEvent() const;
        
    private:
        OverflowEvent();
        OverflowEvent(bool horizontalOverflowChanged, bool horizontalOverflow, bool verticalOverflowChanged, bool verticalOverflow);

        unsigned short m_orient;
        bool m_horizontalOverflow;
        bool m_verticalOverflow;
    };
}

#endif // OverflowEvent_h

