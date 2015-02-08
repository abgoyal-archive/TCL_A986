

#include "config.h"
#include "DragClientQt.h"

#include "ClipboardQt.h"
#include "Frame.h"
#include "PlatformMouseEvent.h"
#include "qwebpage.h"

#include <QDrag>
#include <QMimeData>


namespace WebCore {

static inline Qt::DropActions dragOperationsToDropActions(unsigned op)
{
    Qt::DropActions result = Qt::IgnoreAction;
    if (op & DragOperationCopy)
        result = Qt::CopyAction;
    if (op & DragOperationMove)
        result |= Qt::MoveAction;
    if (op & DragOperationGeneric)
        result |= Qt::MoveAction;
    if (op & DragOperationLink)
        result |= Qt::LinkAction;
    return result;
}

static inline DragOperation dropActionToDragOperation(Qt::DropActions action)
{
    DragOperation result = DragOperationNone;
    if (action & Qt::CopyAction)
        result = DragOperationCopy;
    if (action & Qt::LinkAction)
        result = DragOperationLink;
    if (action & Qt::MoveAction)
        result = DragOperationMove;
    return result;
}

DragDestinationAction DragClientQt::actionMaskForDrag(DragData*)
{
    return DragDestinationActionAny;
}

void DragClientQt::willPerformDragDestinationAction(DragDestinationAction, DragData*)
{
}

void DragClientQt::dragControllerDestroyed()
{
    delete this;
}

DragSourceAction DragClientQt::dragSourceActionMaskForPoint(const IntPoint&)
{
    return DragSourceActionAny;
}

void DragClientQt::willPerformDragSourceAction(DragSourceAction, const IntPoint&, Clipboard*)
{
}

void DragClientQt::startDrag(DragImageRef, const IntPoint&, const IntPoint&, Clipboard* clipboard, Frame* frame, bool)
{
#ifndef QT_NO_DRAGANDDROP
    QMimeData* clipboardData = static_cast<ClipboardQt*>(clipboard)->clipboardData();
    static_cast<ClipboardQt*>(clipboard)->invalidateWritableData();
    QWidget* view = m_webPage->view();
    if (view) {
        QDrag *drag = new QDrag(view);
        if (clipboardData && clipboardData->hasImage())
            drag->setPixmap(qvariant_cast<QPixmap>(clipboardData->imageData()));
        DragOperation dragOperationMask = clipboard->sourceOperation();
        drag->setMimeData(clipboardData);
        Qt::DropAction actualDropAction = drag->exec(dragOperationsToDropActions(dragOperationMask));

        // Send dragEnd event
        PlatformMouseEvent me(m_webPage->view()->mapFromGlobal(QCursor::pos()), QCursor::pos(), LeftButton, MouseEventMoved, 0, false, false, false, false, 0);
        frame->eventHandler()->dragSourceEndedAt(me, dropActionToDragOperation(actualDropAction));
    }
#endif
}


DragImageRef DragClientQt::createDragImageForLink(KURL&, const String&, Frame*)
{
    return 0;
}

} // namespace WebCore
