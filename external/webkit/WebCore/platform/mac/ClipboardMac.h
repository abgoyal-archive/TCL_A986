

#ifndef ClipboardMac_h
#define ClipboardMac_h

#include "CachedResourceClient.h"
#include "Clipboard.h"
#include <wtf/RetainPtr.h>

#ifdef __OBJC__
@class NSImage;
@class NSPasteboard;
#else
class NSImage;
class NSPasteboard;
#endif

namespace WebCore {

class Frame;
class FileList;

class ClipboardMac : public Clipboard, public CachedResourceClient {
public:
    static PassRefPtr<ClipboardMac> create(bool forDragging, NSPasteboard *pasteboard, ClipboardAccessPolicy policy, Frame* frame)
    {
        return adoptRef(new ClipboardMac(forDragging, pasteboard, policy, frame));
    }

    virtual ~ClipboardMac();
    
    void clearData(const String& type);
    void clearAllData();
    String getData(const String& type, bool& success) const;
    bool setData(const String& type, const String& data);
    
    virtual bool hasData();
    
    // extensions beyond IE's API
    virtual HashSet<String> types() const;
    virtual PassRefPtr<FileList> files() const;

    void setDragImage(CachedImage*, const IntPoint&);
    void setDragImageElement(Node *, const IntPoint&);
    
    virtual DragImageRef createDragImage(IntPoint& dragLoc) const;
#if ENABLE(DRAG_SUPPORT)
    virtual void declareAndWriteDragImage(Element*, const KURL&, const String& title, Frame*);
#endif
    virtual void writeRange(Range*, Frame* frame);
    virtual void writeURL(const KURL&, const String&, Frame* frame);
    virtual void writePlainText(const String&);
    
    // Methods for getting info in Cocoa's type system
    NSImage *dragNSImage(NSPoint&) const; // loc converted from dragLoc, based on whole image size
    NSPasteboard *pasteboard() { return m_pasteboard.get(); }

private:
    ClipboardMac(bool forDragging, NSPasteboard *, ClipboardAccessPolicy, Frame*);

    void setDragImage(CachedImage*, Node*, const IntPoint&);

    RetainPtr<NSPasteboard> m_pasteboard;
    int m_changeCount;
    Frame* m_frame; // used on the source side to generate dragging images
};

}

#endif
