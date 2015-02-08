

#include "config.h"
#include "EventHandler.h"

#include "ClipboardHaiku.h"
#include "EventNames.h"
#include "FocusController.h"
#include "Frame.h"
#include "FrameView.h"
#include "HitTestResult.h"
#include "KeyboardEvent.h"
#include "MouseEventWithHitTestResults.h"
#include "NotImplemented.h"
#include "Page.h"
#include "PlatformKeyboardEvent.h"
#include "PlatformScrollBar.h"
#include "PlatformWheelEvent.h"
#include "RenderWidget.h"

#include <interface/View.h>


namespace WebCore {

const double EventHandler::TextDragDelay = 0.0;

static bool isKeyboardOptionTab(KeyboardEvent* event)
{
    return event
        && (event->type() == eventNames().keydownEvent
            || event->type() == eventNames().keypressEvent)
        && event->altKey()
        && event->keyIdentifier() == "U+000009";
}

bool EventHandler::invertSenseOfTabsToLinks(KeyboardEvent* event) const
{
    return isKeyboardOptionTab(event);
}

bool EventHandler::tabsToAllControls(KeyboardEvent* event) const
{
    bool handlingOptionTab = isKeyboardOptionTab(event);

    return handlingOptionTab;
}

void EventHandler::focusDocumentView()
{
    BView* view = m_frame->view()->platformWidget();
    if (view)
        view->MakeFocus();

    Page* page = m_frame->page();
    if (page)
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
    notImplemented();
    return false;
}

bool EventHandler::passSubframeEventToSubframe(MouseEventWithHitTestResults& event, Frame* subframe, HitTestResult*)
{
    notImplemented();
    return true;
}

bool EventHandler::passWheelEventToWidget(PlatformWheelEvent& event, Widget* widget)
{
    if (!widget->isFrameView())
        return false;

    return static_cast<FrameView*>(widget)->frame()->eventHandler()->handleWheelEvent(event);
}

PassRefPtr<Clipboard> EventHandler::createDraggingClipboard() const
{
    return ClipboardHaiku::create(ClipboardWritable, true);
}

bool EventHandler::passMousePressEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
    return passSubframeEventToSubframe(mev, subframe);
}

bool EventHandler::passMouseMoveEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe, HitTestResult* hoveredNode)
{
    return passSubframeEventToSubframe(mev, subframe, hoveredNode);
}

bool EventHandler::passMouseReleaseEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
    return passSubframeEventToSubframe(mev, subframe);
}

unsigned EventHandler::accessKeyModifiers()
{
    return PlatformKeyboardEvent::AltKey;
}

} // namespace WebCore

