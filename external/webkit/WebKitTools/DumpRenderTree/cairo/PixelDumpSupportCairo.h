

#ifndef PixelDumpSupportCairo_h
#define PixelDumpSupportCairo_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

#if PLATFORM(WIN)
#include <windows.h>
#include <cairo-win32.h>
#endif

#if PLATFORM(WIN)
typedef HBITMAP PlatformBitmapBuffer;
#else
typedef void* PlatformBitmapBuffer;
#endif

class BitmapContext : public RefCounted<BitmapContext> {
public:
    static PassRefPtr<BitmapContext> createByAdoptingBitmapAndContext(PlatformBitmapBuffer buffer, cairo_t* context)
    {
        return adoptRef(new BitmapContext(buffer, context));
    }

    ~BitmapContext()
    {
        if (m_buffer)
#if PLATFORM(WIN)
            DeleteObject(m_buffer);
#else
            free(m_buffer);
#endif
        cairo_destroy(m_context);
    }

    cairo_t* cairoContext() const { return m_context; }

private:

    BitmapContext(PlatformBitmapBuffer buffer, cairo_t* context)
        : m_buffer(buffer)
    {
       if (m_context)
          cairo_destroy(m_context);

       m_context = context;
    }

    PlatformBitmapBuffer m_buffer;
    cairo_t* m_context;

};

#endif // PixelDumpSupportCairo_h
