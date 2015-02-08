

#include "config.h"
#include "DOMApplicationCache.h"

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

#include "ApplicationCacheHost.h"
#include "DocumentLoader.h"
#include "Event.h"
#include "EventException.h"
#include "EventListener.h"
#include "EventNames.h"
#include "Frame.h"
#include "FrameLoader.h"

namespace WebCore {

DOMApplicationCache::DOMApplicationCache(Frame* frame)
    : m_frame(frame)
{
    ApplicationCacheHost* cacheHost = applicationCacheHost();
    if (cacheHost)
        cacheHost->setDOMApplicationCache(this);
}

void DOMApplicationCache::disconnectFrame()
{
    ApplicationCacheHost* cacheHost = applicationCacheHost();
    if (cacheHost)
        cacheHost->setDOMApplicationCache(0);
    m_frame = 0;
}

ApplicationCacheHost* DOMApplicationCache::applicationCacheHost() const
{
    if (!m_frame || !m_frame->loader()->documentLoader())
        return 0;
    return m_frame->loader()->documentLoader()->applicationCacheHost();
}

unsigned short DOMApplicationCache::status() const
{
    ApplicationCacheHost* cacheHost = applicationCacheHost();
    if (!cacheHost)
        return ApplicationCacheHost::UNCACHED;
    return cacheHost->status();
}

void DOMApplicationCache::update(ExceptionCode& ec)
{
    ApplicationCacheHost* cacheHost = applicationCacheHost();
    if (!cacheHost || !cacheHost->update())
        ec = INVALID_STATE_ERR;
}

void DOMApplicationCache::swapCache(ExceptionCode& ec)
{
    ApplicationCacheHost* cacheHost = applicationCacheHost();
    if (!cacheHost || !cacheHost->swapCache())
        ec = INVALID_STATE_ERR;
}

ScriptExecutionContext* DOMApplicationCache::scriptExecutionContext() const
{
    if (m_frame)
        return m_frame->document();
    return 0;
}

const AtomicString& DOMApplicationCache::toEventType(ApplicationCacheHost::EventID id)
{
    switch (id) {
    case ApplicationCacheHost::CHECKING_EVENT:
        return eventNames().checkingEvent;
    case ApplicationCacheHost::ERROR_EVENT:
        return eventNames().errorEvent;
    case ApplicationCacheHost::NOUPDATE_EVENT:
        return eventNames().noupdateEvent;
    case ApplicationCacheHost::DOWNLOADING_EVENT:
        return eventNames().downloadingEvent;
    case ApplicationCacheHost::PROGRESS_EVENT:
        return eventNames().progressEvent;
    case ApplicationCacheHost::UPDATEREADY_EVENT:
        return eventNames().updatereadyEvent;
    case ApplicationCacheHost::CACHED_EVENT:
        return eventNames().cachedEvent;
    case ApplicationCacheHost::OBSOLETE_EVENT:            
        return eventNames().obsoleteEvent;
    }
    ASSERT_NOT_REACHED();
    return eventNames().errorEvent;
}

EventTargetData* DOMApplicationCache::eventTargetData()
{
    return &m_eventTargetData;
}

EventTargetData* DOMApplicationCache::ensureEventTargetData()
{
    return &m_eventTargetData;
}

} // namespace WebCore

#endif // ENABLE(OFFLINE_WEB_APPLICATIONS)
