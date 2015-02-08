

#include "config.h"
#include "EventLoop.h"

#include <windows.h>

namespace WebCore {

void EventLoop::cycle()
{
    MSG msg;
    if (!GetMessage(&msg, 0, 0, 0)) {
        m_ended = true;
        return;
    }

    TranslateMessage(&msg);
    DispatchMessage(&msg);
}

} // namespace WebCore
