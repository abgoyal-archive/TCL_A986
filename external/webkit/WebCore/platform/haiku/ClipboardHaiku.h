

#ifndef ClipboardHaiku_h
#define ClipboardHaiku_h

#include "Clipboard.h"
#include "ClipboardAccessPolicy.h"

namespace WebCore {
    class CachedImage;

    // State available during IE's events for drag and drop and copy/paste.
    class ClipboardHaiku : public Clipboard {
    public:
        static PassRefPtr<ClipboardHaiku> create(ClipboardAccessPolicy policy, bool forDragging)
        {
            return adoptRef(new ClipboardHaiku(policy, forDragging));
        }
        ~ClipboardHaiku() { }

        void clearData(const String& type);
        void clearAllData();
        String getData(const String& type, bool& success) const;
        bool setData(const String& type, const String& data);

        // Extensions beyond IE's API.
        HashSet<String> types() const;
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
        virtual void writePlainText(const String&);

        virtual bool hasData();

    private:
        ClipboardHaiku(ClipboardAccessPolicy, bool forDragging);
    };
} // namespace WebCore

#endif // ClipboardHaiku_h

