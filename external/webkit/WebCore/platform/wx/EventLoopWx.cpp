

#include "config.h"
#include "EventLoop.h"

#include <wx/defs.h>
#include <wx/app.h>

namespace WebCore {

void EventLoop::cycle()
{
    if (wxTheApp) {
        if (wxTheApp->IsMainLoopRunning())
            wxTheApp->Dispatch();
        else
            m_ended = true;
    }
}

} // namespace WebCore
