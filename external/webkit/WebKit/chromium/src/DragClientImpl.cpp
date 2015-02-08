

#include "config.h"
#include "DragClientImpl.h"

#include "ChromiumDataObject.h"
#include "ClipboardChromium.h"
#include "Frame.h"
#include "WebDragData.h"
#include "WebViewClient.h"
#include "WebViewImpl.h"

using namespace WebCore;

namespace WebKit {

void DragClientImpl::willPerformDragDestinationAction(DragDestinationAction, DragData*)
{
    // FIXME
}

void DragClientImpl::willPerformDragSourceAction(DragSourceAction, const IntPoint&, Clipboard*)
{
    // FIXME
}

DragDestinationAction DragClientImpl::actionMaskForDrag(DragData*)
{
    if (m_webView->client() && m_webView->client()->acceptsLoadDrops())
        return DragDestinationActionAny;

    return static_cast<DragDestinationAction>(
        DragDestinationActionDHTML | DragDestinationActionEdit);
}

DragSourceAction DragClientImpl::dragSourceActionMaskForPoint(const IntPoint& windowPoint)
{
    // We want to handle drag operations for all source types.
    return DragSourceActionAny;
}

void DragClientImpl::startDrag(DragImageRef dragImage,
                               const IntPoint& dragImageOrigin,
                               const IntPoint& eventPos,
                               Clipboard* clipboard,
                               Frame* frame,
                               bool isLinkDrag)
{
    // Add a ref to the frame just in case a load occurs mid-drag.
    RefPtr<Frame> frameProtector = frame;

    WebDragData dragData = static_cast<ClipboardChromium*>(clipboard)->dataObject();

    DragOperation dragOperationMask = clipboard->sourceOperation();

    m_webView->startDragging(
        eventPos, dragData, static_cast<WebDragOperationsMask>(dragOperationMask));
}

DragImageRef DragClientImpl::createDragImageForLink(KURL&, const String& label, Frame*)
{
    // FIXME
    return 0;
}

void DragClientImpl::dragControllerDestroyed()
{
    // Our lifetime is bound to the WebViewImpl.
}

} // namespace WebKit
