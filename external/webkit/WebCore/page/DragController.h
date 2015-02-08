

#ifndef DragController_h
#define DragController_h

#include "DragActions.h"
#include "DragImage.h"
#include "IntPoint.h"
#include "KURL.h"

namespace WebCore {

    class Clipboard;
    class Document;
    class DragClient;
    class DragData;
    class Element;
    class Frame;
    class Image;
    class IntRect;
    class Node;
    class Page;
    class PlatformMouseEvent;
    class Range;
    class SelectionController;
    
    class DragController : public Noncopyable {
    public:
        DragController(Page*, DragClient*);
        ~DragController();
        DragClient* client() const { return m_client; }

        DragOperation dragEntered(DragData*);
        void dragExited(DragData*);
        DragOperation dragUpdated(DragData*);
        bool performDrag(DragData*);
        
        // FIXME: It should be possible to remove a number of these accessors once all
        // drag logic is in WebCore.
        void setDidInitiateDrag(bool initiated) { m_didInitiateDrag = initiated; } 
        bool didInitiateDrag() const { return m_didInitiateDrag; }
        void setIsHandlingDrag(bool handling) { m_isHandlingDrag = handling; }
        bool isHandlingDrag() const { return m_isHandlingDrag; }
        DragOperation sourceDragOperation() const { return m_sourceDragOperation; }
        void setDraggingImageURL(const KURL& url) { m_draggingImageURL = url; }
        const KURL& draggingImageURL() const { return m_draggingImageURL; }
        void setDragInitiator(Document* initiator) { m_dragInitiator = initiator; m_didInitiateDrag = true; }
        Document* dragInitiator() const { return m_dragInitiator; }
        void setDragOffset(const IntPoint& offset) { m_dragOffset = offset; }
        const IntPoint& dragOffset() const { return m_dragOffset; }
        DragSourceAction dragSourceAction() const { return m_dragSourceAction; }

        Document* documentUnderMouse() const { return m_documentUnderMouse; }
        DragDestinationAction dragDestinationAction() const { return m_dragDestinationAction; }
        DragSourceAction delegateDragSourceAction(const IntPoint& pagePoint);
        
        bool mayStartDragAtEventLocation(const Frame*, const IntPoint& framePos);
        void dragEnded();
        
        void placeDragCaret(const IntPoint&);
        
        bool startDrag(Frame* src, Clipboard*, DragOperation srcOp, const PlatformMouseEvent& dragEvent, const IntPoint& dragOrigin, bool isDHTMLDrag);
        static const IntSize& maxDragImageSize();
        
        static const int LinkDragBorderInset;
        static const int MaxOriginalImageArea;
        static const int DragIconRightInset;
        static const int DragIconBottomInset;        
        static const float DragImageAlpha;

    private:
        bool canProcessDrag(DragData*);
        bool concludeEditDrag(DragData*);
        DragOperation dragEnteredOrUpdated(DragData*);
        DragOperation operationForLoad(DragData*);
        bool tryDocumentDrag(DragData*, DragDestinationAction, DragOperation&);
        bool tryDHTMLDrag(DragData*, DragOperation&);
        DragOperation dragOperation(DragData*);
        void cancelDrag();
        bool dragIsMove(SelectionController*);
        bool isCopyKeyDown();

        void mouseMovedIntoDocument(Document*);

        IntRect selectionDraggingRect(Frame*);
        bool doDrag(Frame* src, Clipboard* clipboard, DragImageRef dragImage, const KURL& linkURL, const KURL& imageURL, Node* node, IntPoint& dragLoc, IntPoint& dragImageOffset);
        void doImageDrag(Element*, const IntPoint&, const IntRect&, Clipboard*, Frame*, IntPoint&);
        void doSystemDrag(DragImageRef, const IntPoint&, const IntPoint&, Clipboard*, Frame*, bool forLink);
        void cleanupAfterSystemDrag();

        Page* m_page;
        DragClient* m_client;
        
        Document* m_documentUnderMouse; // The document the mouse was last dragged over.
        Document* m_dragInitiator; // The Document (if any) that initiated the drag.
        
        DragDestinationAction m_dragDestinationAction;
        DragSourceAction m_dragSourceAction;
        bool m_didInitiateDrag;
        bool m_isHandlingDrag;
        DragOperation m_sourceDragOperation; // Set in startDrag when a drag starts from a mouse down within WebKit
        IntPoint m_dragOffset;
        KURL m_draggingImageURL;
    };

}

#endif
