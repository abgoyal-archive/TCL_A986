

#ifndef WebImage_h
#define WebImage_h

#include "WebCommon.h"

#if WEBKIT_USING_SKIA
#include <SkBitmap.h>
#elif WEBKIT_USING_CG
typedef struct CGImage* CGImageRef;
#endif

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class Image; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {

class WebData;
struct WebSize;

// A container for an ARGB bitmap.
class WebImage {
public:
    ~WebImage() { reset(); }

    WebImage() { init(); }
    WebImage(const WebImage& image)
    {
        init();
        assign(image);
    }

    WebImage& operator=(const WebImage& image)
    {
        assign(image);
        return *this;
    }

    // Decodes the given image data.  If the image has multiple frames,
    // then the frame whose size is desiredSize is returned.  Otherwise,
    // the first frame is returned.
    WEBKIT_API static WebImage fromData(const WebData&, const WebSize& desiredSize);

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebImage&);

    WEBKIT_API bool isNull() const;
    WEBKIT_API WebSize size() const;

#if WEBKIT_IMPLEMENTATION
    WebImage(const WTF::PassRefPtr<WebCore::Image>&);
    WebImage& operator=(const WTF::PassRefPtr<WebCore::Image>&);
#endif

#if WEBKIT_USING_SKIA
    WebImage(const SkBitmap& bitmap) : m_bitmap(bitmap) { }

    WebImage& operator=(const SkBitmap& bitmap)
    {
        m_bitmap = bitmap;
        return *this;
    }

    SkBitmap& getSkBitmap() { return m_bitmap; }
    const SkBitmap& getSkBitmap() const { return m_bitmap; }

private:
    void init() { }
    SkBitmap m_bitmap;

#elif WEBKIT_USING_CG
    WebImage(CGImageRef imageRef)
    {
        init();
        assign(imageRef);
    }

    WebImage& operator=(CGImageRef imageRef)
    {
        assign(imageRef);
        return *this;
    }

    CGImageRef getCGImageRef() const { return m_imageRef; }

private:
    void init() { m_imageRef = 0; }
    void assign(CGImageRef);
    CGImageRef m_imageRef;
#endif
};

} // namespace WebKit

#endif
