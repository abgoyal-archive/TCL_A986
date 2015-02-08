

#ifndef BeforeUnloadEvent_h
#define BeforeUnloadEvent_h

#include "Event.h"

namespace WebCore {

    class BeforeUnloadEvent : public Event {
    public:
        static PassRefPtr<BeforeUnloadEvent> create()
        {
            return adoptRef(new BeforeUnloadEvent);
        }

        virtual bool storesResultAsString() const;
        virtual void storeResult(const String&);

        String result() const { return m_result; }

    private:
        BeforeUnloadEvent();

        String m_result;
    };

} // namespace WebCore

#endif // BeforeUnloadEvent_h
