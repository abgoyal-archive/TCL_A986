

#include "config.h"
#include "EventLoop.h"

#include <app/Application.h>


namespace WebCore {

void EventLoop::cycle()
{
    if (!be_app)
        m_ended = true;
}

} // namespace WebCore

