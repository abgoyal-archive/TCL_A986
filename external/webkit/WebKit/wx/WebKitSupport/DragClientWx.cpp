

#include "config.h"
#include "DragClientWx.h"

#include "NotImplemented.h"

#include <stdio.h>

namespace WebCore {

DragDestinationAction DragClientWx::actionMaskForDrag(DragData*)
{
    notImplemented();
    return DragDestinationActionAny;
}

void DragClientWx::willPerformDragDestinationAction(DragDestinationAction,
                                                    DragData*)
{
    notImplemented();
}

void DragClientWx::willPerformDragSourceAction(DragSourceAction, const IntPoint&, Clipboard*)
{
    notImplemented();
}

void DragClientWx::dragControllerDestroyed()
{
    notImplemented();
}

DragSourceAction DragClientWx::dragSourceActionMaskForPoint(const IntPoint&)
{
    notImplemented();
    return DragSourceActionAny;
}

void DragClientWx::startDrag(DragImageRef dragImage, 
                        const IntPoint& dragImageOrigin, 
                        const IntPoint& eventPos, Clipboard*, 
                        Frame*, bool linkDrag)
{
    notImplemented();
}

DragImageRef DragClientWx::createDragImageForLink(KURL&, const String& label, Frame*)
{
    notImplemented();
    return 0;
}

}
