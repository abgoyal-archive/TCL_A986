

#ifndef DragData_h
#define DragData_h

#include "ClipboardAccessPolicy.h"
#include "Color.h"
#include "DragActions.h"
#include "IntPoint.h"

#include <wtf/Forward.h>
#include <wtf/Vector.h>

#if PLATFORM(MAC)
#ifdef __OBJC__ 
#import <Foundation/Foundation.h>
#import <AppKit/NSDragging.h>
typedef id <NSDraggingInfo> DragDataRef;
#else
typedef void* DragDataRef;
#endif
#elif PLATFORM(QT)
QT_BEGIN_NAMESPACE
class QMimeData;
QT_END_NAMESPACE
typedef const QMimeData* DragDataRef;
#elif PLATFORM(WIN)
typedef struct IDataObject* DragDataRef;
#elif PLATFORM(WX)
typedef class wxDataObject* DragDataRef;
#elif PLATFORM(GTK)
// FIXME: this should probably be something gdk-specific
typedef void* DragDataRef;
#elif defined ANDROID
typedef void* DragDataRef;
#elif PLATFORM(CHROMIUM)
#include "DragDataRef.h"
#elif PLATFORM(HAIKU)
class BMessage;
typedef class BMessage* DragDataRef;
#endif


namespace WebCore {
    
    class Clipboard;
    class Document;
    class DocumentFragment;
    class KURL;
    
#if PLATFORM(MAC)
    class PasteboardHelper;
#endif

    
    class DragData {
    public:
#if PLATFORM(MAC)
        //FIXME: In the future the WebKit functions provided by the helper class should be moved into WebCore, 
        //after which this constructor should be removed
        DragData(DragDataRef data, const IntPoint& clientPosition, const IntPoint& globalPosition,
                 DragOperation operation, PasteboardHelper*);
#else
        //clientPosition is taken to be the position of the drag event within the target window, with (0,0) at the top left
        DragData(DragDataRef data, const IntPoint& clientPosition, const IntPoint& globalPosition, DragOperation operation);
#endif
        const IntPoint& clientPosition() const { return m_clientPosition; }
        const IntPoint& globalPosition() const { return m_globalPosition; }
        DragDataRef platformData() const { return m_platformDragData; }
        DragOperation draggingSourceOperationMask() const { return m_draggingSourceOperationMask; }
        PassRefPtr<Clipboard> createClipboard(ClipboardAccessPolicy) const;
        bool containsURL() const;
        bool containsPlainText() const;
        bool containsCompatibleContent() const;
        String asURL(String* title = 0) const;
        String asPlainText() const;
        void asFilenames(Vector<String>&) const;
        Color asColor() const;
        PassRefPtr<DocumentFragment> asFragment(Document*) const;
        bool canSmartReplace() const;
        bool containsColor() const;
        bool containsFiles() const;
    private:
        IntPoint m_clientPosition;
        IntPoint m_globalPosition;
        DragDataRef m_platformDragData;
        DragOperation m_draggingSourceOperationMask;
#if PLATFORM(MAC)
        PasteboardHelper* m_pasteboardHelper;
#endif
};
    
} //namespace WebCore

#endif //!DragData_h

