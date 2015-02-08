

#include "config.h"

#if ENABLE(TOUCH_EVENTS)

#include "Touch.h"

#include "FrameView.h"

namespace WebCore {

static int contentsX(Frame* frame)
{
    if (!frame)
        return 0;
    FrameView* frameView = frame->view();
    if (!frameView)
        return 0;
    return frameView->scrollX() / frame->pageZoomFactor();
}

static int contentsY(Frame* frame)
{
    if (!frame)
        return 0;
    FrameView* frameView = frame->view();
    if (!frameView)
        return 0;
    return frameView->scrollY() / frame->pageZoomFactor();
}

Touch::Touch(Frame* frame, EventTarget* target, unsigned identifier, 
        int screenX, int screenY, int pageX, int pageY)
    : m_target(target)
    , m_identifier(identifier)
    , m_clientX(pageX - contentsX(frame))
    , m_clientY(pageY - contentsY(frame))
    , m_screenX(screenX)
    , m_screenY(screenY)
    , m_pageX(pageX)
    , m_pageY(pageY)
{
}

} // namespace WebCore

#endif

