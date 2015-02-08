

#ifndef ClipboardGtk_h
#define ClipboardGtk_h

#include "Clipboard.h"

namespace WebCore {
    class CachedImage;

    // State available during IE's events for drag and drop and copy/paste
    // Created from the EventHandlerGtk to be used by the dom
    class ClipboardGtk : public Clipboard {
    public:
        static PassRefPtr<ClipboardGtk> create(ClipboardAccessPolicy policy, bool isForDragging)
        {
            return adoptRef(new ClipboardGtk(policy, isForDragging));
        }
        virtual ~ClipboardGtk();

        void clearData(const String&);
        void clearAllData();
        String getData(const String&, bool&) const;
        bool setData(const String&, const String&);

        virtual HashSet<String> types() const;
        virtual PassRefPtr<FileList> files() const;

        IntPoint dragLocation() const;
        CachedImage* dragImage() const;
        void setDragImage(CachedImage*, const IntPoint&);
        Node* dragImageElement();
        void setDragImageElement(Node*, const IntPoint&);

        virtual DragImageRef createDragImage(IntPoint&) const;
        virtual void declareAndWriteDragImage(Element*, const KURL&, const String&, Frame*);
        virtual void writeURL(const KURL&, const String&, Frame*);
        virtual void writeRange(Range*, Frame*);
        virtual void writePlainText(const String&);

        virtual bool hasData();

    private:
        ClipboardGtk(ClipboardAccessPolicy, bool);
    };
}

#endif
