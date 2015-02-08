

#ifndef PixelDumpSupportCG_h
#define PixelDumpSupportCG_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RetainPtr.h>

#if PLATFORM(WIN)
#include <windows.h>
#endif

typedef struct CGContext* CGContextRef;

#if PLATFORM(MAC)
typedef void* PlatformBitmapBuffer;
#elif PLATFORM(WIN)
typedef HBITMAP PlatformBitmapBuffer;
#endif

class BitmapContext : public RefCounted<BitmapContext> {
public:
    static PassRefPtr<BitmapContext> createByAdoptingBitmapAndContext(PlatformBitmapBuffer buffer, CGContextRef context)
    {
        return adoptRef(new BitmapContext(buffer, context));
    }

    ~BitmapContext()
    {
        if (m_buffer)
#if PLATFORM(MAC)
            free(m_buffer);
#elif PLATFORM(WIN)
            DeleteObject(m_buffer);
#endif
    }

    CGContextRef cgContext() const { return m_context.get(); }

private:

    BitmapContext(PlatformBitmapBuffer buffer, CGContextRef context)
        : m_buffer(buffer)
        , m_context(AdoptCF, context)
    {
    }

    PlatformBitmapBuffer m_buffer;
    RetainPtr<CGContextRef> m_context;

};

PassRefPtr<BitmapContext> createBitmapContextFromWebView(bool onscreen, bool incrementalRepaint, bool sweepHorizontally, bool drawSelectionRect);

#endif // PixelDumpSupportCG_h
