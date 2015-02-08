

#include "config.h"
#include "EventHandler.h"

#include "ClipboardWin.h"
#include "Cursor.h"
#include "FloatPoint.h"
#include "FocusController.h"
#include "FrameView.h"
#include "Frame.h"
#include "HitTestRequest.h"
#include "HitTestResult.h"
#include "MouseEventWithHitTestResults.h"
#include "Page.h"
#include "PlatformKeyboardEvent.h"
#include "PlatformWheelEvent.h"
#include "Scrollbar.h"
#include "SelectionController.h"
#include "WCDataObject.h"
#include "NotImplemented.h"

namespace WebCore {

const double EventHandler::TextDragDelay = 0.0;

bool EventHandler::passMousePressEventToSubframe(MouseEventWithHitTestResults& mev, Frame* subframe)
{
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
    if (!widget->isFrameView())
        return false;

    return static_cast<FrameView*>(widget)->frame()->eventHandler()->handleWheelEvent(wheelEvent);
}

bool EventHandler::tabsToAllControls(KeyboardEvent*) const
{
    return true;
}

bool EventHandler::eventActivatedView(const PlatformMouseEvent& event) const
{
    return event.activatedWebView();
}

PassRefPtr<Clipboard> EventHandler::createDraggingClipboard() const
{
#if OS(WINCE)
    return 0;
#else
    COMPtr<WCDataObject> dataObject;
    WCDataObject::createInstance(&dataObject);
    return ClipboardWin::create(true, dataObject.get(), ClipboardWritable);
#endif
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

unsigned EventHandler::accessKeyModifiers()
{
    return PlatformKeyboardEvent::AltKey;
}

}
