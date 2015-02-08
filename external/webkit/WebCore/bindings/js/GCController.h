

#ifndef GCController_h
#define GCController_h

#include <wtf/Noncopyable.h>
#include "Timer.h"

namespace WebCore {

    class GCController : public Noncopyable {
        friend GCController& gcController();

    public:
        void garbageCollectSoon();
        void garbageCollectNow(); // It's better to call garbageCollectSoon, unless you have a specific reason not to.

        void garbageCollectOnAlternateThreadForDebugging(bool waitUntilDone); // Used for stress testing.

    private:
        GCController(); // Use gcController() instead
        void gcTimerFired(Timer<GCController>*);
        
        Timer<GCController> m_GCTimer;
    };

    // Function to obtain the global GC controller.
    GCController& gcController();

} // namespace WebCore

#endif // GCController_h
