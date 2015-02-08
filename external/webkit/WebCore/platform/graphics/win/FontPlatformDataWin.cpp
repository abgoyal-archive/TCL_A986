

#include "config.h"
#include "FontPlatformData.h"

#include "PlatformString.h"
#include "StringHash.h"
#include <wtf/HashMap.h>
#include <wtf/RetainPtr.h>
#include <wtf/Vector.h>

using std::min;

namespace WebCore {

FontPlatformData::FontPlatformData(HFONT font, float size, bool bold, bool oblique, bool useGDI)
    : m_font(RefCountedHFONT::create(font))
    , m_size(size)
#if PLATFORM(CG)
    , m_cgFont(0)
#elif PLATFORM(CAIRO)
    , m_fontFace(0)
    , m_scaledFont(0)
#endif
    , m_syntheticBold(bold)
    , m_syntheticOblique(oblique)
    , m_useGDI(useGDI)
{
    HDC hdc = GetDC(0);
    SaveDC(hdc);
    
    SelectObject(hdc, font);
    UINT bufferSize = GetOutlineTextMetrics(hdc, 0, NULL);

    ASSERT_WITH_MESSAGE(bufferSize, "Bitmap fonts not supported with CoreGraphics.");

    if (bufferSize) {
        OUTLINETEXTMETRICW* metrics = (OUTLINETEXTMETRICW*)malloc(bufferSize);

        GetOutlineTextMetricsW(hdc, bufferSize, metrics);
        WCHAR* faceName = (WCHAR*)((uintptr_t)metrics + (uintptr_t)metrics->otmpFaceName);

        platformDataInit(font, size, hdc, faceName);

        free(metrics);
    }

    RestoreDC(hdc, -1);
    ReleaseDC(0, hdc);
}

FontPlatformData::FontPlatformData(float size, bool bold, bool oblique)
    : m_size(size)
#if PLATFORM(CG)
    , m_cgFont(0)
#elif PLATFORM(CAIRO)
    , m_fontFace(0)
    , m_scaledFont(0)
#endif
    , m_syntheticBold(bold)
    , m_syntheticOblique(oblique)
    , m_useGDI(false)
{
}

#ifndef NDEBUG
String FontPlatformData::description() const
{
    return String();
}
#endif

}
