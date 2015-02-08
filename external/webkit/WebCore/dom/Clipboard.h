

#ifndef Clipboard_h
#define Clipboard_h

#include "CachedResourceHandle.h"
#include "ClipboardAccessPolicy.h"
#include "DragActions.h"
#include "DragImage.h"
#include "IntPoint.h"
#include "Node.h"

namespace WebCore {

    class FileList;

    // State available during IE's events for drag and drop and copy/paste
    class Clipboard : public RefCounted<Clipboard> {
    public:
        virtual ~Clipboard() { }

        // Is this operation a drag-drop or a copy-paste?
        bool isForDragging() const { return m_forDragging; }

        String dropEffect() const { return m_dropEffect; }
        void setDropEffect(const String&);
        String effectAllowed() const { return m_effectAllowed; }
        void setEffectAllowed(const String&);
    
        virtual void clearData(const String& type) = 0;
        virtual void clearAllData() = 0;
        virtual String getData(const String& type, bool& success) const = 0;
        virtual bool setData(const String& type, const String& data) = 0;
    
        // extensions beyond IE's API
        virtual HashSet<String> types() const = 0;
        virtual PassRefPtr<FileList> files() const = 0;

        IntPoint dragLocation() const { return m_dragLoc; }
        CachedImage* dragImage() const { return m_dragImage.get(); }
        virtual void setDragImage(CachedImage*, const IntPoint&) = 0;
        Node* dragImageElement() const { return m_dragImageElement.get(); }
        virtual void setDragImageElement(Node*, const IntPoint&) = 0;
        
        virtual DragImageRef createDragImage(IntPoint& dragLocation) const = 0;
#if ENABLE(DRAG_SUPPORT)
        virtual void declareAndWriteDragImage(Element*, const KURL&, const String& title, Frame*) = 0;
#endif
        virtual void writeURL(const KURL&, const String&, Frame*) = 0;
        virtual void writeRange(Range*, Frame*) = 0;
        virtual void writePlainText(const String&) = 0;

        virtual bool hasData() = 0;
        
        void setAccessPolicy(ClipboardAccessPolicy);

        DragOperation sourceOperation() const;
        DragOperation destinationOperation() const;
        void setSourceOperation(DragOperation);
        void setDestinationOperation(DragOperation);
        
        void setDragHasStarted() { m_dragStarted = true; }
        
    protected:
        Clipboard(ClipboardAccessPolicy, bool isForDragging);

        ClipboardAccessPolicy policy() const { return m_policy; }
        bool dragStarted() const { return m_dragStarted; }
        
    private:
        ClipboardAccessPolicy m_policy;
        String m_dropEffect;
        String m_effectAllowed;
        bool m_dragStarted;
        
    protected:
        bool m_forDragging;
        IntPoint m_dragLoc;
        CachedResourceHandle<CachedImage> m_dragImage;
        RefPtr<Node> m_dragImageElement;
    };

} // namespace WebCore

#endif // Clipboard_h
