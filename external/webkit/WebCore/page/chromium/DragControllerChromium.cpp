

#include "config.h"
#include "DragController.h"

#include "DragData.h"
#include "SelectionController.h"
#include <wtf/RefPtr.h>

#if OS(WINDOWS)
#include <windows.h>
#endif

namespace WebCore {

const int DragController::LinkDragBorderInset = 2;
const int DragController::MaxOriginalImageArea = 1500 * 1500;
const int DragController::DragIconRightInset = 7;
const int DragController::DragIconBottomInset = 3;

const float DragController::DragImageAlpha = 0.75f;

DragOperation DragController::dragOperation(DragData* dragData)
{
    // FIXME: To match the MacOS behaviour we should return DragOperationNone
    // if we are a modal window, we are the drag source, or the window is an
    // attached sheet If this can be determined from within WebCore
    // operationForDrag can be pulled into WebCore itself
    ASSERT(dragData);
    return dragData->containsURL() && !m_didInitiateDrag ? DragOperationCopy : DragOperationNone;
}

bool DragController::isCopyKeyDown()
{
    // FIXME: This should not be OS specific.  Delegate to the embedder instead.
#if OS(WINDOWS)
    return ::GetAsyncKeyState(VK_CONTROL);
#else
    return false;
#endif
}
    
const IntSize& DragController::maxDragImageSize()
{
    static const IntSize maxDragImageSize(200, 200);
    return maxDragImageSize;
}

void DragController::cleanupAfterSystemDrag()
{
}

} // namespace WebCore
