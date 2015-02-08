

#ifndef ClipboardWin_h
#define ClipboardWin_h

#include "CachedResourceClient.h"
#include "Clipboard.h"
#include "COMPtr.h"

struct IDataObject;

namespace WebCore {

    class CachedImage;
    class IntPoint;
    class WCDataObject;

    // State available during IE's events for drag and drop and copy/paste
    class ClipboardWin : public Clipboard, public CachedResourceClient {
    public:
        static PassRefPtr<ClipboardWin> create(bool isForDragging, IDataObject* dataObject, ClipboardAccessPolicy policy)
        {
            return adoptRef(new ClipboardWin(isForDragging, dataObject, policy));
        }
        static PassRefPtr<ClipboardWin> create(bool isForDragging, WCDataObject* dataObject, ClipboardAccessPolicy policy)
        {
            return adoptRef(new ClipboardWin(isForDragging, dataObject, policy));
        }
        ~ClipboardWin();
    
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

        COMPtr<IDataObject> dataObject() { return m_dataObject; }

        void setExternalDataObject(IDataObject *dataObject);

    private:
        ClipboardWin(bool isForDragging, IDataObject*, ClipboardAccessPolicy);
        ClipboardWin(bool isForDragging, WCDataObject*, ClipboardAccessPolicy);

        void resetFromClipboard();
        void setDragImage(CachedImage*, Node*, const IntPoint&);

        COMPtr<IDataObject> m_dataObject;
        COMPtr<WCDataObject> m_writableDataObject;
        Frame* m_frame;
    };

} // namespace WebCore

#endif // ClipboardWin_h
