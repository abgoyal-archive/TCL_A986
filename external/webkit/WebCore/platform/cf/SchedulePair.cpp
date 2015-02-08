

#include "config.h"
#include "SchedulePair.h"

namespace WebCore {

SchedulePair::SchedulePair(CFRunLoopRef runLoop, CFStringRef mode)
    : m_runLoop(runLoop)
{
    if (mode)
        m_mode.adoptCF(CFStringCreateCopy(0, mode));
}

bool SchedulePair::operator==(const SchedulePair& other) const
{
    if (runLoop() != other.runLoop())
        return false;
    CFStringRef thisMode = mode();
    CFStringRef otherMode = other.mode();
    if (!thisMode || !otherMode)
        return thisMode == otherMode;
    return CFEqual(thisMode, otherMode);
}

} // namespace
