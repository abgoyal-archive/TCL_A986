

#include "config.h"
#include "NavigationAction.h"

#include "Event.h"
#include "FrameLoader.h"

namespace WebCore {

static NavigationType navigationType(FrameLoadType frameLoadType, bool isFormSubmission, bool haveEvent)
{
    if (isFormSubmission)
        return NavigationTypeFormSubmitted;
    if (haveEvent)
        return NavigationTypeLinkClicked;
    if (frameLoadType == FrameLoadTypeReload || frameLoadType == FrameLoadTypeReloadFromOrigin)
        return NavigationTypeReload;
    if (isBackForwardLoadType(frameLoadType))
        return NavigationTypeBackForward;
    return NavigationTypeOther;
}

NavigationAction::NavigationAction()
    : m_type(NavigationTypeOther)
{
}

NavigationAction::NavigationAction(const KURL& url, NavigationType type)
    : m_URL(url)
    , m_type(type)
{
}

NavigationAction::NavigationAction(const KURL& url, FrameLoadType frameLoadType,
        bool isFormSubmission)
    : m_URL(url)
    , m_type(navigationType(frameLoadType, isFormSubmission, 0))
{
}

NavigationAction::NavigationAction(const KURL& url, NavigationType type, PassRefPtr<Event> event)
    : m_URL(url)
    , m_type(type)
    , m_event(event)
{
}

NavigationAction::NavigationAction(const KURL& url, FrameLoadType frameLoadType,
        bool isFormSubmission, PassRefPtr<Event> event)
    : m_URL(url)
    , m_type(navigationType(frameLoadType, isFormSubmission, event))
    , m_event(event)
{
}

}
