

#ifndef NavigationAction_h
#define NavigationAction_h

#include "Event.h"
#include "FrameLoaderTypes.h"
#include "KURL.h"
#include <wtf/Forward.h>

namespace WebCore {

    class NavigationAction {
    public:
        NavigationAction();
        NavigationAction(const KURL&, NavigationType);
        NavigationAction(const KURL&, FrameLoadType, bool isFormSubmission);
        NavigationAction(const KURL&, NavigationType, PassRefPtr<Event>);
        NavigationAction(const KURL&, FrameLoadType, bool isFormSubmission, PassRefPtr<Event>);

        bool isEmpty() const { return m_URL.isEmpty(); }

        KURL url() const { return m_URL; }
        NavigationType type() const { return m_type; }
        const Event* event() const { return m_event.get(); }

    private:
        KURL m_URL;
        NavigationType m_type;
        RefPtr<Event> m_event;
    };

}

#endif
