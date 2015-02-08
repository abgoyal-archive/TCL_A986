

#include "DragClient.h"
#include "DragData.h"

namespace WebCore {

class DragClientWx : public WebCore::DragClient {
public:
    virtual void willPerformDragDestinationAction(WebCore::DragDestinationAction,
                                                  WebCore::DragData*);
    virtual void willPerformDragSourceAction(WebCore::DragSourceAction, const WebCore::IntPoint&, WebCore::Clipboard*);

    virtual WebCore::DragDestinationAction actionMaskForDrag(WebCore::DragData*);
    virtual void dragControllerDestroyed();
    virtual WebCore::DragSourceAction dragSourceActionMaskForPoint(const WebCore::IntPoint&);
        
    virtual void startDrag(WebCore::DragImageRef dragImage, const WebCore::IntPoint& dragImageOrigin, const WebCore::IntPoint& eventPos, WebCore::Clipboard*, WebCore::Frame*, bool linkDrag = false);
    virtual WebCore::DragImageRef createDragImageForLink(WebCore::KURL&, const WebCore::String& label, WebCore::Frame*);
};

}
