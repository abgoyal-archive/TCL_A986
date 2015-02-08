

#include "config.h"
#include "EventLoop.h"

#include <QCoreApplication>

namespace WebCore {

void EventLoop::cycle()
{
    QCoreApplication::processEvents();
}

} // namespace WebCore
