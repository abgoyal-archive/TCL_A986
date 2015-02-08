

#ifndef Image_h
#define Image_h

#include "Color.h"
#include "ColorSpace.h"
#include "GraphicsTypes.h"
#include "ImageSource.h"
#include "IntRect.h"
#include "SharedBuffer.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

#if PLATFORM(MAC)
#ifdef __OBJC__
@class NSImage;
#else
class NSImage;
#endif
#endif

#if PLATFORM(CG)
struct CGContext;
#endif

#if PLATFORM(WIN)
typedef struct tagSIZE SIZE;
typedef SIZE* LPSIZE;
typedef struct HBITMAP__ *HBITMAP;
#endif

#if PLATFORM(SKIA)
class NativeImageSkia;
#endif

#if PLATFORM(QT)
#include <QPixmap>
#endif

#if PLATFORM(GTK)
typedef struct _GdkPixbuf GdkPixbuf;
#endif

namespace WebCore {

class FloatPoint;
class FloatRect;
class FloatSize;
class GraphicsContext;
class SharedBuffer;
class String;
class AffineTransform;

// This class gets notified when an image creates or destroys decoded frames and when it advances animation frames.
class ImageObserver;

class Image : public RefCounted<Image> {
    friend class GeneratedImage;
    friend class GraphicsContext;

public:
    virtual ~Image();
    
    static PassRefPtr<Image> create(ImageObserver* = 0);
    static PassRefPtr<Image> loadPlatformResource(const char* name);
    static bool supportsType(const String&); 

    virtual bool isBitmapImage() const { return false; }

    // Derived classes should override this if they can assure that 
    // the image contains only resources from its own security origin.
    virtual bool hasSingleSecurityOrigin() const { return false; }

    static Image* nullImage();
    bool isNull() const { return size().isEmpty(); }

    // These are only used for SVGImage right now
    virtual void setContainerSize(const IntSize&) { }
    virtual bool usesContainerSize() const { return false; }
    virtual bool hasRelativeWidth() const { return false; }
    virtual bool hasRelativeHeight() const { return false; }

    virtual IntSize size() const = 0;
    IntRect rect() const { return IntRect(IntPoint(), size()); }
    int width() const { return size().width(); }
    int height() const { return size().height(); }

    bool setData(PassRefPtr<SharedBuffer> data, bool allDataReceived);
    virtual bool dataChanged(bool /*allDataReceived*/) { return false; }
    
    virtual String filenameExtension() const { return String(); } // null string if unknown

    virtual void destroyDecodedData(bool destroyAll = true) = 0;
    virtual unsigned decodedSize() const = 0;

    SharedBuffer* data() { return m_data.get(); }

    // Animation begins whenever someone draws the image, so startAnimation() is not normally called.
    // It will automatically pause once all observers no longer want to render the image anywhere.
    virtual void startAnimation(bool /*catchUpIfNecessary*/ = true) { }
    virtual void stopAnimation() {}
    virtual void resetAnimation() {}
    
    // Typically the CachedImage that owns us.
    ImageObserver* imageObserver() const { return m_imageObserver; }

    enum TileRule { StretchTile, RoundTile, RepeatTile };

    virtual NativeImagePtr nativeImageForCurrentFrame() { return 0; }
    
#if PLATFORM(MAC)
    // Accessors for native image formats.
    virtual NSImage* getNSImage() { return 0; }
    virtual CFDataRef getTIFFRepresentation() { return 0; }
#endif

#if PLATFORM(CG)
    virtual CGImageRef getCGImageRef() { return 0; }
#endif

#if PLATFORM(WIN)
    virtual bool getHBITMAP(HBITMAP) { return false; }
    virtual bool getHBITMAPOfSize(HBITMAP, LPSIZE) { return false; }
#endif

#if PLATFORM(ANDROID)
    virtual void setURL(const String& str) {}
#endif

#if PLATFORM(GTK)
    virtual GdkPixbuf* getGdkPixbuf() { return 0; }
    static PassRefPtr<Image> loadPlatformThemeIcon(const char* name, int size);
#endif

protected:
    Image(ImageObserver* = 0);

    static void fillWithSolidColor(GraphicsContext*, const FloatRect& dstRect, const Color&, ColorSpace styleColorSpace, CompositeOperator);

    // The ColorSpace parameter will only be used for untagged images.
#if PLATFORM(WIN)
    virtual void drawFrameMatchingSourceSize(GraphicsContext*, const FloatRect& dstRect, const IntSize& srcSize, ColorSpace styleColorSpace, CompositeOperator) { }
#endif
    virtual void draw(GraphicsContext*, const FloatRect& dstRect, const FloatRect& srcRect, ColorSpace styleColorSpace, CompositeOperator) = 0;
    void drawTiled(GraphicsContext*, const FloatRect& dstRect, const FloatPoint& srcPoint, const FloatSize& tileSize, ColorSpace styleColorSpace, CompositeOperator);
    void drawTiled(GraphicsContext*, const FloatRect& dstRect, const FloatRect& srcRect, TileRule hRule, TileRule vRule, ColorSpace styleColorSpace, CompositeOperator);

    // Supporting tiled drawing
    virtual bool mayFillWithSolidColor() { return false; }
    virtual Color solidColor() const { return Color(); }
    
    virtual void drawPattern(GraphicsContext*, const FloatRect& srcRect, const AffineTransform& patternTransform,
                             const FloatPoint& phase, ColorSpace styleColorSpace, CompositeOperator, const FloatRect& destRect);

private:
    RefPtr<SharedBuffer> m_data; // The encoded raw data for the image. 
    ImageObserver* m_imageObserver;
};

}

#endif
