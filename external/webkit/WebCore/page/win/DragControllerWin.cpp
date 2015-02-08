

#include "config.h"
#include "DragController.h"

#include "DragData.h"
#include "windows.h" 
#include "SelectionController.h"
#include <wtf/RefPtr.h>

namespace WebCore {

const int DragController::LinkDragBorderInset = 2;
const int DragController::MaxOriginalImageArea = 1500 * 1500;
const int DragController::DragIconRightInset = 7;
const int DragController::DragIconBottomInset = 3;

const float DragController::DragImageAlpha = 0.75f;

DragOperation DragController::dragOperation(DragData* dragData)
{
    //FIXME: to match the macos behaviour we should return DragOperationNone
    //if we are a modal window, we are the drag source, or the window is an attached sheet
    //If this can be determined from within WebCore operationForDrag can be pulled into 
    //WebCore itself
    ASSERT(dragData);
    return dragData->containsURL() && !m_didInitiateDrag ? DragOperationCopy : DragOperationNone;
}

bool DragController::isCopyKeyDown()
{
    return ::GetAsyncKeyState(VK_CONTROL);
}
    
const IntSize& DragController::maxDragImageSize()
{
    static const IntSize maxDragImageSize(200, 200);
    
    return maxDragImageSize;
}

void DragController::cleanupAfterSystemDrag()
{
}

}
