

#include "config.h"
#include "EventHandler.h"

#include "ChromiumDataObject.h"
#include "ClipboardChromium.h"
#include "Cursor.h"
#include "FloatPoint.h"
#include "FocusController.h"
#include "FrameView.h"
#include "Frame.h"
#include "HitTestRequest.h"
#include "HitTestResult.h"
#include "MouseEventWithHitTestResults.h"
#include "NotImplemented.h"
#include "Page.h"
#include "PlatformKeyboardEvent.h"
#include "PlatformWheelEvent.h"
#include "RenderWidget.h"
#include "SelectionController.h"

namespace WebCore {

#if OS(DARWIN)
const double EventHandler::TextDragDelay = 0.15;
#else
const double EventHandler::TextDragDelay = 0.0;
#endif

bool EventHandler::passMousePressEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
    // If we're clicking into a frame that is selected, the frame will appear
    // greyed out even though we're clicking on the selection.  This looks
    // really strange (having the whole frame be greyed out), so we deselect the
    // selection.
    IntPoint p = m_frame->view()->windowToContents(mev.event().pos());
    if (m_frame->selection()->contains(p)) {
        VisiblePosition visiblePos(
            mev.targetNode()->renderer()->positionForPoint(mev.localPoint()));
        VisibleSelection newSelection(visiblePos);
        if (m_frame->shouldChangeSelection(newSelection))
            m_frame->selection()->setSelection(newSelection);
    }

    subframe->eventHandler()->handleMousePressEvent(mev.event());
    return true;
}

bool EventHandler::passMouseMoveEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe, HitTestResult* hoveredNode)
{
    if (m_mouseDownMayStartDrag && !m_mouseDownWasInSubframe)
        return false;
    subframe->eventHandler()->handleMouseMoveEvent(mev.event(), hoveredNode);
    return true;
}

bool EventHandler::passMouseReleaseEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
    subframe->eventHandler()->handleMouseReleaseEvent(mev.event());
    return true;
}

bool EventHandler::passWheelEventToWidget(PlatformWheelEvent& wheelEvent, Widget* widget)
{
    // We can sometimes get a null widget!  EventHandlerMac handles a null
    // widget by returning false, so we do the same.
    if (!widget)
        return false;

    // If not a FrameView, then probably a plugin widget.  Those will receive
    // the event via an EventTargetNode dispatch when this returns false.
    if (!widget->isFrameView())
        return false;

    return static_cast<FrameView*>(widget)->frame()->eventHandler()->handleWheelEvent(wheelEvent);
}

bool EventHandler::passWidgetMouseDownEventToWidget(const MouseEventWithHitTestResults& event)
{
    // Figure out which view to send the event to.
    if (!event.targetNode() || !event.targetNode()->renderer() || !event.targetNode()->renderer()->isWidget())
        return false;
    return passMouseDownEventToWidget(toRenderWidget(event.targetNode()->renderer())->widget());
}

bool EventHandler::passMouseDownEventToWidget(Widget* widget)
{
    notImplemented();
    return false;
}

bool EventHandler::tabsToAllControls(KeyboardEvent*) const
{
    return true;
}

bool EventHandler::eventActivatedView(const PlatformMouseEvent& event) const
{
    // FIXME: EventHandlerWin.cpp does the following:
    // return event.activatedWebView();
    return false;
}

PassRefPtr<Clipboard> EventHandler::createDraggingClipboard() const
{
    RefPtr<ChromiumDataObject> dataObject = ChromiumDataObject::create();
    return ClipboardChromium::create(true, dataObject.get(), ClipboardWritable);
}

void EventHandler::focusDocumentView()
{
    Page* page = m_frame->page();
    if (!page)
        return;
    page->focusController()->setFocusedFrame(m_frame);
}

bool EventHandler::passWidgetMouseDownEventToWidget(RenderWidget* renderWidget)
{
    return passMouseDownEventToWidget(renderWidget->widget());
}

unsigned EventHandler::accessKeyModifiers()
{
#if OS(DARWIN)
    return PlatformKeyboardEvent::CtrlKey | PlatformKeyboardEvent::AltKey;
#else
    return PlatformKeyboardEvent::AltKey;
#endif
}

#if OS(LINUX)
// GTK+ must scroll horizontally if the mouse pointer is on top of the
// horizontal scrollbar while scrolling with the wheel.
// This code comes from gtk/EventHandlerGtk.cpp.
bool EventHandler::shouldTurnVerticalTicksIntoHorizontal(const HitTestResult& result) const
{
    return result.scrollbar() && result.scrollbar()->orientation() == HorizontalScrollbar;
}
#endif

} // namespace WebCore
