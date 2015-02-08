

#include "config.h"
#include "EventLoop.h"

#include <glib.h>

namespace WebCore {

void EventLoop::cycle()
{
    g_main_context_iteration(NULL, FALSE);
}

} // namespace WebCore
