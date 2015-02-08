

#include "config.h"
#include "DragClientHaiku.h"

#include "NotImplemented.h"


namespace WebCore {

DragDestinationAction DragClientHaiku::actionMaskForDrag(DragData*)
{
    notImplemented();
    return DragDestinationActionAny;
}

void DragClientHaiku::willPerformDragDestinationAction(DragDestinationAction, DragData*)
{
    notImplemented();
}

void DragClientHaiku::dragControllerDestroyed()
{
    notImplemented();
}

DragSourceAction DragClientHaiku::dragSourceActionMaskForPoint(const IntPoint&)
{
    notImplemented();
    return DragSourceActionAny;
}

void DragClientHaiku::willPerformDragSourceAction(DragSourceAction, const IntPoint&, Clipboard*)
{
    notImplemented();
}

void DragClientHaiku::startDrag(DragImageRef dragImage, const IntPoint&, const IntPoint&, Clipboard*, Frame*, bool)
{
    notImplemented();
}

DragImageRef DragClientHaiku::createDragImageForLink(KURL&, const String&, Frame*)
{
    notImplemented();
    return 0;
}

} // namespace WebCore

