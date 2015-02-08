

#ifndef ClipboardAndroid_h
#define ClipboardAndroid_h

#include "Clipboard.h"

#include "CachedResourceClient.h"

namespace WebCore {

class CachedImage;

class ClipboardAndroid : public Clipboard, public CachedResourceClient {
public:
    ClipboardAndroid(ClipboardAccessPolicy policy, bool isForDragging);
    ~ClipboardAndroid();

    void clearData(const String&);
    void clearAllData();
    String getData(const String&, bool& success) const;
    bool setData(const String&, const String&);

    // extensions beyond IE's API
    HashSet<String> types() const;
    PassRefPtr<FileList> files() const;

    void setDragImage(CachedImage*, const IntPoint&);
    void setDragImageElement(Node*, const IntPoint&);
    
    virtual DragImageRef createDragImage(IntPoint&) const;
    virtual void declareAndWriteDragImage(Element*, const KURL&, const String&, Frame*);
    virtual void writeURL(const KURL&, const String&, Frame*);
    virtual void writeRange(Range*, Frame*);
    virtual void writePlainText(const String&);

    virtual bool hasData();
};

} // namespace WebCore

#endif // ClipboardAndroid_h
