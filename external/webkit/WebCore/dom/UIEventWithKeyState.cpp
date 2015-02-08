

#include "config.h"
#include "UIEventWithKeyState.h"

namespace WebCore {
    
UIEventWithKeyState* findEventWithKeyState(Event* event)
{
    for (Event* e = event; e; e = e->underlyingEvent())
        if (e->isKeyboardEvent() || e->isMouseEvent())
            return static_cast<UIEventWithKeyState*>(e);
    return 0;
}

} // namespace WebCore
