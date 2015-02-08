

#include "config.h"
#include "FontPlatformData.h"

#include "HarfbuzzSkia.h"
#include "NotImplemented.h"
#include "PlatformString.h"
#include "StringImpl.h"

#include "SkPaint.h"
#include "SkTypeface.h"

namespace WebCore {

static SkPaint::Hinting skiaHinting = SkPaint::kNormal_Hinting;
static bool isSkiaAntiAlias = true, isSkiaSubpixelGlyphs;

void FontPlatformData::setHinting(SkPaint::Hinting hinting)
{
    skiaHinting = hinting;
}

void FontPlatformData::setAntiAlias(bool isAntiAlias)
{
    isSkiaAntiAlias = isAntiAlias;
}

void FontPlatformData::setSubpixelGlyphs(bool isSubpixelGlyphs)
{
    isSkiaSubpixelGlyphs = isSubpixelGlyphs;
}

FontPlatformData::RefCountedHarfbuzzFace::~RefCountedHarfbuzzFace()
{
    HB_FreeFace(m_harfbuzzFace);
}

FontPlatformData::FontPlatformData(const FontPlatformData& src)
    : m_typeface(src.m_typeface)
    , m_textSize(src.m_textSize)
    , m_fakeBold(src.m_fakeBold)
    , m_fakeItalic(src.m_fakeItalic)
    , m_harfbuzzFace(src.m_harfbuzzFace)
{
    m_typeface->safeRef();
}

FontPlatformData::FontPlatformData(SkTypeface* tf, float textSize, bool fakeBold, bool fakeItalic)
    : m_typeface(tf)
    , m_textSize(textSize)
    , m_fakeBold(fakeBold)
    , m_fakeItalic(fakeItalic)
{
    m_typeface->safeRef();
}

FontPlatformData::FontPlatformData(const FontPlatformData& src, float textSize)
    : m_typeface(src.m_typeface)
    , m_textSize(textSize)
    , m_fakeBold(src.m_fakeBold)
    , m_fakeItalic(src.m_fakeItalic)
    , m_harfbuzzFace(src.m_harfbuzzFace)
{
    m_typeface->safeRef();
}

FontPlatformData::~FontPlatformData()
{
    m_typeface->safeUnref();
}

FontPlatformData& FontPlatformData::operator=(const FontPlatformData& src)
{
    SkRefCnt_SafeAssign(m_typeface, src.m_typeface);

    m_textSize = src.m_textSize;
    m_fakeBold = src.m_fakeBold;
    m_fakeItalic = src.m_fakeItalic;
    m_harfbuzzFace = src.m_harfbuzzFace;

    return *this;
}

#ifndef NDEBUG
String FontPlatformData::description() const
{
    return String();
}
#endif

void FontPlatformData::setupPaint(SkPaint* paint) const
{
    const float ts = m_textSize > 0 ? m_textSize : 12;

    paint->setAntiAlias(isSkiaAntiAlias);
    paint->setHinting(skiaHinting);
    paint->setLCDRenderText(isSkiaSubpixelGlyphs);
    paint->setTextSize(SkFloatToScalar(ts));
    paint->setTypeface(m_typeface);
    paint->setFakeBoldText(m_fakeBold);
    paint->setTextSkewX(m_fakeItalic ? -SK_Scalar1 / 4 : 0);
}

SkFontID FontPlatformData::uniqueID() const
{
    return m_typeface->uniqueID();
}

bool FontPlatformData::operator==(const FontPlatformData& a) const
{
    // If either of the typeface pointers are invalid (either NULL or the
    // special deleted value) then we test for pointer equality. Otherwise, we
    // call SkTypeface::Equal on the valid pointers.
    bool typefacesEqual;
    if (m_typeface == hashTableDeletedFontValue()
        || a.m_typeface == hashTableDeletedFontValue()
        || !m_typeface
        || !a.m_typeface)
        typefacesEqual = m_typeface == a.m_typeface;
    else
        typefacesEqual = SkTypeface::Equal(m_typeface, a.m_typeface);

    return typefacesEqual 
        && m_textSize == a.m_textSize
        && m_fakeBold == a.m_fakeBold
        && m_fakeItalic == a.m_fakeItalic;
}

unsigned FontPlatformData::hash() const
{
    unsigned h = SkTypeface::UniqueID(m_typeface);
    h ^= 0x01010101 * ((static_cast<int>(m_fakeBold) << 1) | static_cast<int>(m_fakeItalic));

    // This memcpy is to avoid a reinterpret_cast that breaks strict-aliasing
    // rules. Memcpy is generally optimized enough so that performance doesn't
    // matter here.
    uint32_t textSizeBytes;
    memcpy(&textSizeBytes, &m_textSize, sizeof(uint32_t));
    h ^= textSizeBytes;

    return h;
}

bool FontPlatformData::isFixedPitch() const
{
    notImplemented();
    return false;
}

HB_FaceRec_* FontPlatformData::harfbuzzFace() const
{
    if (!m_harfbuzzFace)
        m_harfbuzzFace = RefCountedHarfbuzzFace::create(HB_NewFace(const_cast<FontPlatformData*>(this), harfbuzzSkiaGetTable));

    return m_harfbuzzFace->face();
}

}  // namespace WebCore
