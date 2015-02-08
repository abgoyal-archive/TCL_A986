

#include "config.h"

#if ENABLE(WORKERS)

#include "WorkerNavigator.h"

namespace WebCore {

WorkerNavigator::WorkerNavigator(const String& userAgent)
    : m_userAgent(userAgent)
{
}

WorkerNavigator::~WorkerNavigator()
{
}

String WorkerNavigator::userAgent() const
{
    return m_userAgent;
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
