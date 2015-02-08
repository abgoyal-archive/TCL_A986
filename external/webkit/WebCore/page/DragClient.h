


#ifndef DragClient_h
#define DragClient_h

#include "DragActions.h"
#include "DragImage.h"
#include "IntPoint.h"

#if PLATFORM(MAC)
#ifdef __OBJC__
@class DOMElement;
@class NSURL;
@class NSString;
@class NSPasteboard;
#else
class DOMElement;
class NSURL;
class NSString;
class NSPasteboard;
#endif
#endif

namespace WebCore {
    
    class Clipboard;
    class DragData;
    class Frame;
    class Image;
    class HTMLImageElement;
    
    class DragClient {
    public:
        virtual void willPerformDragDestinationAction(DragDestinationAction, DragData*) = 0;
        virtual void willPerformDragSourceAction(DragSourceAction, const IntPoint&, Clipboard*) = 0;
        virtual DragDestinationAction actionMaskForDrag(DragData*) = 0;
        //We work in window rather than view coordinates here
        virtual DragSourceAction dragSourceActionMaskForPoint(const IntPoint& windowPoint) = 0;
        
        virtual void startDrag(DragImageRef dragImage, const IntPoint& dragImageOrigin, const IntPoint& eventPos, Clipboard*, Frame*, bool linkDrag = false) = 0;
        virtual DragImageRef createDragImageForLink(KURL&, const String& label, Frame*) = 0;
        
        virtual void dragControllerDestroyed() = 0;
#if PLATFORM(MAC)
        //Mac specific helper functions to allow access to functionality in webkit -- such as 
        //web archives and NSPasteboard extras 
        //not abstract as that would require another #if PLATFORM(MAC) for the SVGImage client empty impl
        virtual void declareAndWriteDragImage(NSPasteboard*, DOMElement*, NSURL*, NSString*, Frame*) {};
#endif
        
        virtual ~DragClient() {};
    };
    
}

#endif // !DragClient_h

