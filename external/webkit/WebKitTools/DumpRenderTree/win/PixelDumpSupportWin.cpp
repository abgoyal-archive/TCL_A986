

#include "config.h"

#if PLATFORM(CG)
#include "PixelDumpSupportCG.h"
#elif PLATFORM(CAIRO)
#include "PixelDumpSupportCairo.h"
#endif

#include "DumpRenderTree.h"

#if PLATFORM(CG)
// Note: Must be included *after* DumpRenderTree.h to avoid compile error.
#include <CoreGraphics/CGBitmapContext.h>
#endif

#include <wtf/Assertions.h>
#include <wtf/RetainPtr.h>

PassRefPtr<BitmapContext> createBitmapContextFromWebView(bool onscreen, bool incrementalRepaint, bool sweepHorizontally, bool drawSelectionRect)
{
    RECT frame;
    if (!GetWindowRect(webViewWindow, &frame))
        return 0;

    BITMAPINFO bmp = {0};
    bmp.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bmp.bmiHeader.biWidth = frame.right - frame.left;
    bmp.bmiHeader.biHeight = -(frame.bottom - frame.top);
    bmp.bmiHeader.biPlanes = 1;
    bmp.bmiHeader.biBitCount = 32;
    bmp.bmiHeader.biCompression = BI_RGB;

    void* bits = 0;
    HBITMAP bitmap = CreateDIBSection(0, &bmp, DIB_RGB_COLORS, &bits, 0, 0);

    HDC memoryDC = CreateCompatibleDC(0);
    SelectObject(memoryDC, bitmap);
    SendMessage(webViewWindow, WM_PRINTCLIENT, reinterpret_cast<WPARAM>(memoryDC), PRF_CLIENT | PRF_CHILDREN | PRF_OWNED);
    DeleteDC(memoryDC);

    BITMAP info = {0};
    GetObject(bitmap, sizeof(info), &info);
    ASSERT(info.bmBitsPixel == 32);

#if PLATFORM(CG)
    RetainPtr<CGColorSpaceRef> colorSpace(AdoptCF, CGColorSpaceCreateDeviceRGB());
    CGContextRef context = CGBitmapContextCreate(info.bmBits, info.bmWidth, info.bmHeight, 8,
                                                info.bmWidthBytes, colorSpace.get(), kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
#elif PLATFORM(CAIRO) 
    cairo_surface_t* image = cairo_image_surface_create_for_data((unsigned char*)info.bmBits, CAIRO_FORMAT_ARGB32, 
                                                      info.bmWidth, info.bmHeight, info.bmWidthBytes); 
    cairo_t* context = cairo_create(image); 
    cairo_surface_destroy(image); 
#endif 

   return BitmapContext::createByAdoptingBitmapAndContext(bitmap, context);
}
