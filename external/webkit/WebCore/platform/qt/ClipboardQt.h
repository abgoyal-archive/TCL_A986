

#ifndef ClipboardQt_h
#define ClipboardQt_h

#include "Clipboard.h"
#include "CachedResourceClient.h"

QT_BEGIN_NAMESPACE
class QMimeData;
QT_END_NAMESPACE

namespace WebCore {

    class CachedImage;

    // State available during IE's events for drag and drop and copy/paste
    class ClipboardQt : public Clipboard, public CachedResourceClient {
    public:
        static PassRefPtr<ClipboardQt> create(ClipboardAccessPolicy policy, const QMimeData* readableClipboard)
        {
            return adoptRef(new ClipboardQt(policy, readableClipboard));
        }
        static PassRefPtr<ClipboardQt> create(ClipboardAccessPolicy policy, bool forDragging = false)
        {
            return adoptRef(new ClipboardQt(policy, forDragging));
        }
        virtual ~ClipboardQt();

        void clearData(const String& type);
        void clearAllData();
        String getData(const String& type, bool& success) const;
        bool setData(const String& type, const String& data);

        // extensions beyond IE's API
        virtual HashSet<String> types() const;
        virtual PassRefPtr<FileList> files() const;

        void setDragImage(CachedImage*, const IntPoint&);
        void setDragImageElement(Node*, const IntPoint&);

        virtual DragImageRef createDragImage(IntPoint& dragLoc) const;
        virtual void declareAndWriteDragImage(Element*, const KURL&, const String& title, Frame*);
        virtual void writeURL(const KURL&, const String&, Frame*);
        virtual void writeRange(Range*, Frame*);
        virtual void writePlainText(const String&);

        virtual bool hasData();

        QMimeData* clipboardData() const { return m_writableData; }
        void invalidateWritableData() { m_writableData = 0; }

    private:
        ClipboardQt(ClipboardAccessPolicy, const QMimeData* readableClipboard);

        // Clipboard is writable so it will create its own QMimeData object
        ClipboardQt(ClipboardAccessPolicy, bool forDragging);

        void setDragImage(CachedImage*, Node*, const IntPoint& loc);

        const QMimeData* m_readableData;
        QMimeData* m_writableData;
    };
}

#endif // ClipboardQt_h
