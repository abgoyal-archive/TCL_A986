

#include "config.h"
#include "EventHandler.h"

#include "ClipboardQt.h"
#include "Cursor.h"
#include "Document.h"
#include "EventNames.h"
#include "FloatPoint.h"
#include "FocusController.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "FrameTree.h"
#include "FrameView.h"
#include "HTMLFrameSetElement.h"
#include "HitTestRequest.h"
#include "HitTestResult.h"
#include "KeyboardEvent.h"
#include "MouseEventWithHitTestResults.h"
#include "Page.h"
#include "PlatformKeyboardEvent.h"
#include "PlatformWheelEvent.h"
#include "RenderWidget.h"
#include "Scrollbar.h"
#include "NotImplemented.h"

QT_BEGIN_NAMESPACE
extern Q_GUI_EXPORT bool qt_tab_all_widgets; // from qapplication.cpp
QT_END_NAMESPACE

namespace WebCore {

const double EventHandler::TextDragDelay = 0.0;

static bool isKeyboardOptionTab(KeyboardEvent* event)
{
    return event
        && (event->type() == eventNames().keydownEvent || event->type() == eventNames().keypressEvent)
        && event->altKey()
        && event->keyIdentifier() == "U+0009";
}

bool EventHandler::invertSenseOfTabsToLinks(KeyboardEvent* event) const
{
    return isKeyboardOptionTab(event);
}

bool EventHandler::tabsToAllControls(KeyboardEvent* event) const
{
    return (isKeyboardOptionTab(event) ? !qt_tab_all_widgets : qt_tab_all_widgets);
}

void EventHandler::focusDocumentView()
{
    Page* page = m_frame->page();
    if (!page)
        return;
    page->focusController()->setFocusedFrame(m_frame);
}

bool EventHandler::passWidgetMouseDownEventToWidget(const MouseEventWithHitTestResults&)
{
    notImplemented();
    return false;
}

bool EventHandler::eventActivatedView(const PlatformMouseEvent&) const
{
    //Qt has an activation event which is sent independently
    //   of mouse event so this thing will be a snafu to implement
    //   correctly
    return false;
}

bool EventHandler::passWheelEventToWidget(PlatformWheelEvent& event, Widget* widget)
{
    Q_ASSERT(widget);
    if (!widget->isFrameView())
        return false;

    return static_cast<FrameView*>(widget)->frame()->eventHandler()->handleWheelEvent(event);
}

PassRefPtr<Clipboard> EventHandler::createDraggingClipboard() const
{
    return ClipboardQt::create(ClipboardWritable, true);
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
    return PlatformKeyboardEvent::CtrlKey;
}

}
