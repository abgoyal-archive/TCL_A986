

#include "config.h"
#include "History.h"

#include "ExceptionCode.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "FrameLoaderClient.h"
#include "HistoryItem.h"
#include "Page.h"

namespace WebCore {

History::History(Frame* frame)
    : m_frame(frame)
{
}

Frame* History::frame() const
{
    return m_frame;
}

void History::disconnectFrame()
{
    m_frame = 0;
}

unsigned History::length() const
{
    if (!m_frame)
        return 0;
    if (!m_frame->page())
        return 0;
    return m_frame->page()->getHistoryLength();
}

void History::back()
{
    if (!m_frame)
        return;
    m_frame->redirectScheduler()->scheduleHistoryNavigation(-1);
}

void History::forward()
{
    if (!m_frame)
        return;
    m_frame->redirectScheduler()->scheduleHistoryNavigation(1);
}

void History::go(int distance)
{
    if (!m_frame)
        return;
    m_frame->redirectScheduler()->scheduleHistoryNavigation(distance);
}

KURL History::urlForState(const String& urlString)
{
    KURL baseURL = m_frame->loader()->baseURL();
    if (urlString.isEmpty())
        return baseURL;
        
    return KURL(baseURL, urlString);
}

void History::stateObjectAdded(PassRefPtr<SerializedScriptValue> data, const String& title, const String& urlString, StateObjectType stateObjectType, ExceptionCode& ec)
{
    if (!m_frame || !m_frame->page())
        return;
    
    KURL fullURL = urlForState(urlString);
    RefPtr<SecurityOrigin> origin = SecurityOrigin::create(fullURL);
    if (!fullURL.isValid() || !m_frame->document()->securityOrigin()->isSameSchemeHostPort(origin.get())) {
        ec = SECURITY_ERR;
        return;
    }

    if (stateObjectType == StateObjectPush)
        m_frame->loader()->history()->pushState(data, title, fullURL.string());
    else if (stateObjectType == StateObjectReplace)
        m_frame->loader()->history()->replaceState(data, title, fullURL.string());
            
    if (!urlString.isEmpty())
        m_frame->document()->updateURLForPushOrReplaceState(fullURL);

    if (stateObjectType == StateObjectPush)
        m_frame->loader()->client()->dispatchDidPushStateWithinPage();
    else if (stateObjectType == StateObjectReplace)
        m_frame->loader()->client()->dispatchDidReplaceStateWithinPage();
}

} // namespace WebCore
