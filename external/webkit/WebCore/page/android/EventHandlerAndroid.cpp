
#define LOG_TAG "WebCore"

#include "config.h"
#include "EventHandler.h"

#include "FocusController.h"
#include "Frame.h"
#include "KeyboardEvent.h"
#include "MouseEventWithHitTestResults.h"
#include "Page.h"
#include "PlatformKeyboardEvent.h"
#include "PlatformWheelEvent.h"
#include "RenderWidget.h"

namespace WebCore {

bool EventHandler::tabsToAllControls(KeyboardEvent*) const
{
    return true;
}

void EventHandler::focusDocumentView()
{
    if (Page* page = m_frame->page())
        page->focusController()->setFocusedFrame(m_frame);
}

bool EventHandler::passWidgetMouseDownEventToWidget(const MouseEventWithHitTestResults& event)
{
    // Figure out which view to send the event to.
    RenderObject* target = event.targetNode() ? event.targetNode()->renderer() : 0;
    if (!target || !target->isWidget())
        return false;
    return passMouseDownEventToWidget(toRenderWidget(target)->widget());
}

bool EventHandler::passWidgetMouseDownEventToWidget(RenderWidget* renderWidget)
{
    return passMouseDownEventToWidget(renderWidget->widget());
}

// This function is used to route the mouse down event to the native widgets, it seems like a
// work around for the Mac platform which does not support double clicks, but browsers do.
bool EventHandler::passMouseDownEventToWidget(Widget*)
{
    // return false so the normal propogation handles the event
    return false;
}

bool EventHandler::eventActivatedView(const PlatformMouseEvent&) const
{
    notImplemented();
    return false;
}

// This is called to route wheel events to child widgets when they are RenderWidget
// as the parent usually gets wheel event. Don't have a mouse with a wheel to confirm
// the operation of this function.
bool EventHandler::passWheelEventToWidget(PlatformWheelEvent&, Widget*)
{
    notImplemented();
    return false;
}

bool EventHandler::passMousePressEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
    subframe->eventHandler()->handleMousePressEvent(mev.event());
    return true;
}

bool EventHandler::passMouseMoveEventToSubframe(MouseEventWithHitTestResults& mev, 
    Frame* subframe, HitTestResult* hoveredNode)
{
    subframe->eventHandler()->handleMouseMoveEvent(mev.event(), hoveredNode);
    return true;
}

bool EventHandler::passMouseReleaseEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
    subframe->eventHandler()->handleMouseReleaseEvent(mev.event());
    return true;
}

class Clipboard : public RefCounted<Clipboard> {
};

PassRefPtr<Clipboard> EventHandler::createDraggingClipboard() const
{
    return PassRefPtr<Clipboard>(0);
}

unsigned EventHandler::accessKeyModifiers()
{
    return PlatformKeyboardEvent::AltKey;
}

const double EventHandler::TextDragDelay = 0.0;

}  // namespace WebCore
