

#ifndef DragClientAndroid_h
#define DragClientAndroid_h

#include "DragClient.h"

using namespace WebCore;

namespace android {

    class DragClientAndroid : public DragClient {
    public:
        virtual void willPerformDragDestinationAction(DragDestinationAction, DragData*);
        virtual void willPerformDragSourceAction(DragSourceAction, const IntPoint&, Clipboard*);
        virtual DragDestinationAction actionMaskForDrag(DragData*);
        //We work in window rather than view coordinates here
        virtual DragSourceAction dragSourceActionMaskForPoint(const IntPoint&);
        
        virtual void startDrag(DragImageRef dragImage, const IntPoint& dragImageOrigin, const IntPoint& eventPos, Clipboard*, Frame*, bool linkDrag = false);
        virtual DragImageRef createDragImageForLink(KURL&, const String& label, Frame*);

        virtual void dragControllerDestroyed();
    };
        
}

#endif
