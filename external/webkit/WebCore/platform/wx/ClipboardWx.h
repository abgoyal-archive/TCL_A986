

#ifndef ClipboardWx_h
#define ClipboardWx_h

#include "Clipboard.h"

namespace WebCore {

    class CachedImage;

    // State available during IE's events for drag and drop and copy/paste
    class ClipboardWx : public Clipboard {
    public:
        static PassRefPtr<ClipboardWx> create(ClipboardAccessPolicy policy, bool forDragging)
        {
            return adoptRef(new ClipboardWx(policy, forDragging));
        }

        void clearData(const String& type);
        void clearAllData();
        String getData(const String& type, bool& success) const;
        bool setData(const String& type, const String& data);
    
        // extensions beyond IE's API
        virtual HashSet<String> types() const;
        virtual PassRefPtr<FileList> files() const;
    
        IntPoint dragLocation() const;
        CachedImage* dragImage() const;
        void setDragImage(CachedImage*, const IntPoint&);
        Node* dragImageElement();
        void setDragImageElement(Node*, const IntPoint&);

        virtual DragImageRef createDragImage(IntPoint& dragLoc) const;
        virtual void declareAndWriteDragImage(Element*, const KURL&, const String& title, Frame*);
        virtual void writeURL(const KURL&, const String&, Frame*);
        virtual void writeRange(Range*, Frame*);
        virtual void writePlainText(const WebCore::String&);


        virtual bool hasData();

    private:
        ClipboardWx(ClipboardAccessPolicy, bool forDragging);
    };
} 

#endif // ClipboardWx_h
