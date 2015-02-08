

#include "config.h"
#include "EventHandler.h"

#include "ClipboardGtk.h"
#include "FloatPoint.h"
#include "FocusController.h"
#include "Frame.h"
#include "FrameView.h"
#include "KeyboardEvent.h"
#include "MouseEventWithHitTestResults.h"
#include "NotImplemented.h"
#include "Page.h"
#include "PlatformKeyboardEvent.h"
#include "PlatformWheelEvent.h"
#include "RenderWidget.h"
#include "Scrollbar.h"

namespace WebCore {

const double EventHandler::TextDragDelay = 0.0;

bool EventHandler::tabsToAllControls(KeyboardEvent* event) const
{
    // We always allow tabs to all controls
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

bool EventHandler::passMouseDownEventToWidget(Widget* widget)
{
    notImplemented();
    return false;
}

bool EventHandler::eventActivatedView(const PlatformMouseEvent&) const
{
    //GTK+ activation is not necessarily tied to mouse events, so it may
    //not make sense to implement this

    notImplemented();
    return false;
}

bool EventHandler::passWheelEventToWidget(PlatformWheelEvent& event, Widget* widget)
{
    ASSERT(widget);
    if (!widget->isFrameView())
        return false;

    return static_cast<FrameView*>(widget)->frame()->eventHandler()->handleWheelEvent(event);
}

PassRefPtr<Clipboard> EventHandler::createDraggingClipboard() const
{
    return ClipboardGtk::create(ClipboardWritable, true);
}

bool EventHandler::passMousePressEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
    subframe->eventHandler()->handleMousePressEvent(mev.event());
    return true;
}

bool EventHandler::passMouseMoveEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe, HitTestResult* hoveredNode)
{
    subframe->eventHandler()->handleMouseMoveEvent(mev.event(), hoveredNode);
    return true;
}

bool EventHandler::passMouseReleaseEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
    subframe->eventHandler()->handleMouseReleaseEvent(mev.event());
    return true;
}

unsigned EventHandler::accessKeyModifiers()
{
    return PlatformKeyboardEvent::AltKey;
}

// GTK+ must scroll horizontally if the mouse pointer is on top of the
// horizontal scrollbar while scrolling with the wheel; we need to
// add the deltas and ticks here so that this behavior is consistent
// for styled scrollbars.
bool EventHandler::shouldTurnVerticalTicksIntoHorizontal(const HitTestResult& result) const
{
    return result.scrollbar() && result.scrollbar()->orientation() == HorizontalScrollbar;
}

}
