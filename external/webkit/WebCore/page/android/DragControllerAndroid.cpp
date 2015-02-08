

#include "config.h"
#include "DragController.h"

#include "DragData.h"
#include "NotImplemented.h"

namespace WebCore {

bool DragController::isCopyKeyDown()
{
    return false;
}
    
DragOperation DragController::dragOperation(DragData* dragData)
{
    // FIXME: This logic is incomplete
    notImplemented();
    if (dragData->containsURL())
        return DragOperationCopy;

    return DragOperationNone;
}

void DragController::cleanupAfterSystemDrag()
{
}

const float DragController::DragImageAlpha = 1.0f;
static IntSize dummy;
const IntSize& DragController::maxDragImageSize() { return dummy; }
const int DragController::DragIconRightInset = 0;
const int DragController::DragIconBottomInset = 0;
const int DragController::LinkDragBorderInset = 0;
const int DragController::MaxOriginalImageArea = 0;

}  // namespace WebCore
