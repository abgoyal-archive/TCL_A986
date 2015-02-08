

#include "config.h"
#include "EventHandler.h"

#include "ClipboardWx.h"
#include "FocusController.h"
#include "Frame.h"
#include "FrameView.h"
#include "KeyboardEvent.h"
#include "MouseEventWithHitTestResults.h"
#include "NotImplemented.h"
#include "Page.h"
#include "PlatformKeyboardEvent.h"
#include "RenderWidget.h"
#include "Scrollbar.h"

namespace WebCore {

const double EventHandler::TextDragDelay = 0.0;

bool EventHandler::passMousePressEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
    subframe->eventHandler()->handleMousePressEvent(mev.event());
    return true;
}

bool EventHandler::passMouseMoveEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe, WebCore::HitTestResult* hittest)
{
    subframe->eventHandler()->handleMouseMoveEvent(mev.event(), hittest);
    return true;
}

bool EventHandler::passMouseReleaseEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
    subframe->eventHandler()->handleMouseReleaseEvent(mev.event());
    return true;
}

bool EventHandler::passWidgetMouseDownEventToWidget(const MouseEventWithHitTestResults& event)
{
    // Figure out which view to send the event to.
    if (!event.targetNode() || !event.targetNode()->renderer() || !event.targetNode()->renderer()->isWidget())
        return false;
    
    return passMouseDownEventToWidget(toRenderWidget(event.targetNode()->renderer())->widget());
}

bool EventHandler::passWidgetMouseDownEventToWidget(RenderWidget* renderWidget)
{
    return passMouseDownEventToWidget(renderWidget->widget());
}

bool EventHandler::passWheelEventToWidget(PlatformWheelEvent& event, Widget* widget)
{
    if (!widget || !widget->isFrameView())
        return false;

    return static_cast<FrameView*>(widget)->frame()->eventHandler()->handleWheelEvent(event);
}

bool EventHandler::tabsToAllControls(KeyboardEvent* event) const 
{ 
    notImplemented(); 
    return false; 
}

bool EventHandler::passSubframeEventToSubframe(MouseEventWithHitTestResults&, Frame* subframe, HitTestResult*)
{
    notImplemented(); 
    return false; 
}

bool EventHandler::passMouseDownEventToWidget(Widget*)
{ 
    notImplemented();
    return false; 
}

void EventHandler::focusDocumentView()
{
    if (Page* page = m_frame->page())
        page->focusController()->setFocusedFrame(m_frame);
}

bool EventHandler::eventActivatedView(const PlatformMouseEvent&) const
{
    // wx sends activate events separate from mouse events. 
    // We'll have to test the exact order of events, 
    // but for the moment just return false.
    return false;
}

PassRefPtr<Clipboard> EventHandler::createDraggingClipboard() const 
{
    return ClipboardWx::create(ClipboardWritable, true);
}

unsigned EventHandler::accessKeyModifiers()
{
    return PlatformKeyboardEvent::AltKey;
}

}
