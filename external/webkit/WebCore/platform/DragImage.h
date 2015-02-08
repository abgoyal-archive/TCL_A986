

#ifndef DragImage_h
#define DragImage_h

#include "IntSize.h"
#include "FloatSize.h"

#if PLATFORM(MAC)
#include <wtf/RetainPtr.h>
#ifdef __OBJC__
@class NSImage;
#else
class NSImage;
#endif
#elif PLATFORM(QT)
QT_BEGIN_NAMESPACE
class QPixmap;
QT_END_NAMESPACE
#elif PLATFORM(WIN)
typedef struct HBITMAP__* HBITMAP;
#elif PLATFORM(WX)
class wxDragImage;
#elif PLATFORM(CHROMIUM)
#include "DragImageRef.h"
#elif PLATFORM(GTK)
typedef struct _GdkPixbuf GdkPixbuf;
#elif PLATFORM(HAIKU)
class BBitmap;
#elif PLATFORM(BREWMP)
typedef struct IImage IImage;
#endif

//We need to #define YOffset as it needs to be shared with WebKit
#define DragLabelBorderYOffset 2

namespace WebCore {
    
    class CachedImage;
    class Frame;
    class Image;
    class KURL;
    class Range;
    class String;
    
#if PLATFORM(MAC)
    typedef RetainPtr<NSImage> DragImageRef;
#elif PLATFORM(QT)
    typedef QPixmap* DragImageRef;
#elif PLATFORM(WIN)
    typedef HBITMAP DragImageRef;
#elif PLATFORM(WX)
    typedef wxDragImage* DragImageRef;
#elif PLATFORM(GTK)
    typedef GdkPixbuf* DragImageRef;
#elif PLATFORM(HAIKU)
    typedef BBitmap* DragImageRef;
#elif PLATFORM(BREWMP)
    typedef IImage* DragImageRef;
#elif PLATFORM(ANDROID)
    typedef void* DragImageRef;
#endif
    
    IntSize dragImageSize(DragImageRef);
    
    //These functions should be memory neutral, eg. if they return a newly allocated image, 
    //they should release the input image.  As a corollary these methods don't guarantee
    //the input image ref will still be valid after they have been called
    DragImageRef fitDragImageToMaxSize(DragImageRef image, const IntSize& srcSize, const IntSize& size);
    DragImageRef scaleDragImage(DragImageRef, FloatSize scale);
    DragImageRef dissolveDragImageToFraction(DragImageRef image, float delta);
    
    DragImageRef createDragImageFromImage(Image*);
    DragImageRef createDragImageForSelection(Frame*);    
    DragImageRef createDragImageIconForCachedImage(CachedImage*);
    void deleteDragImage(DragImageRef);
}


#endif //!DragImage_h
