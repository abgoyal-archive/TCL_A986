

#include "config.h"
#include "SimpleFontData.h"

#include "Font.h"
#include "FontCache.h"
#include "FloatRect.h"
#include "FontDescription.h"
#include "Logging.h"
#include "VDMXParser.h"

#include "SkFontHost.h"
#include "SkPaint.h"
#include "SkTime.h"
#include "SkTypeface.h"
#include "SkTypes.h"

namespace WebCore {

// Smallcaps versions of fonts are 70% the size of the normal font.
static const float smallCapsFraction = 0.7f;
// This is the largest VDMX table which we'll try to load and parse.
static const size_t maxVDMXTableSize = 1024 * 1024;  // 1 MB

void SimpleFontData::platformInit()
{
    SkPaint paint;
    SkPaint::FontMetrics metrics;

    m_platformData.setupPaint(&paint);
    paint.getFontMetrics(&metrics);
    const SkFontID fontID = m_platformData.uniqueID();

    static const uint32_t vdmxTag = SkSetFourByteTag('V', 'D', 'M', 'X');
    int pixelSize = m_platformData.size() + 0.5;
    int vdmxAscent, vdmxDescent;
    bool isVDMXValid = false;

    size_t vdmxSize = SkFontHost::GetTableSize(fontID, vdmxTag);
    if (vdmxSize && vdmxSize < maxVDMXTableSize) {
        uint8_t* vdmxTable = (uint8_t*) fastMalloc(vdmxSize);
        if (vdmxTable
            && SkFontHost::GetTableData(fontID, vdmxTag, 0, vdmxSize, vdmxTable) == vdmxSize
            && parseVDMX(&vdmxAscent, &vdmxDescent, vdmxTable, vdmxSize, pixelSize))
            isVDMXValid = true;
        fastFree(vdmxTable);
    }

    // Beware those who step here: This code is designed to match Win32 font
    // metrics *exactly*.
    if (isVDMXValid) {
        m_ascent = vdmxAscent;
        m_descent = -vdmxDescent;
    } else {
        SkScalar height = -metrics.fAscent + metrics.fDescent + metrics.fLeading;
        m_ascent = SkScalarRound(-metrics.fAscent);
        m_descent = SkScalarRound(height) - m_ascent;
    }

    if (metrics.fXHeight)
        m_xHeight = metrics.fXHeight;
    else {
        // hack taken from the Windows port
        m_xHeight = static_cast<float>(m_ascent) * 0.56;
    }

    m_lineGap = SkScalarRound(metrics.fLeading);
    m_lineSpacing = m_ascent + m_descent + m_lineGap;

    // In WebKit/WebCore/platform/graphics/SimpleFontData.cpp, m_spaceWidth is
    // calculated for us, but we need to calculate m_maxCharWidth and
    // m_avgCharWidth in order for text entry widgets to be sized correctly.

    SkScalar xRange = metrics.fXMax - metrics.fXMin;
    m_maxCharWidth = SkScalarRound(xRange * SkScalarRound(m_platformData.size()));

    if (metrics.fAvgCharWidth)
        m_avgCharWidth = SkScalarRound(metrics.fAvgCharWidth);
    else {
        m_avgCharWidth = m_xHeight;

        GlyphPage* glyphPageZero = GlyphPageTreeNode::getRootChild(this, 0)->page();

        if (glyphPageZero) {
            static const UChar32 x_char = 'x';
            const Glyph xGlyph = glyphPageZero->glyphDataForCharacter(x_char).glyph;

            if (xGlyph)
                m_avgCharWidth = widthForGlyph(xGlyph);
        }
    }
}

void SimpleFontData::platformCharWidthInit()
{
    // charwidths are set in platformInit.
}

void SimpleFontData::platformDestroy()
{
    delete m_smallCapsFontData;
    m_smallCapsFontData = 0;
}

SimpleFontData* SimpleFontData::smallCapsFontData(const FontDescription& fontDescription) const
{
    if (!m_smallCapsFontData) {
        const float smallCapsSize = lroundf(fontDescription.computedSize() * smallCapsFraction);
        m_smallCapsFontData = new SimpleFontData(FontPlatformData(m_platformData, smallCapsSize));
    }

    return m_smallCapsFontData;
}

bool SimpleFontData::containsCharacters(const UChar* characters, int length) const
{
    SkPaint paint;
    static const unsigned maxBufferCount = 64;
    uint16_t glyphs[maxBufferCount];

    m_platformData.setupPaint(&paint);
    paint.setTextEncoding(SkPaint::kUTF16_TextEncoding);

    while (length > 0) {
        int n = SkMin32(length, SK_ARRAY_COUNT(glyphs));

        // textToGlyphs takes a byte count so we double the character count.
        int count = paint.textToGlyphs(characters, n * 2, glyphs);
        for (int i = 0; i < count; i++) {
            if (0 == glyphs[i])
                return false;       // missing glyph
        }

        characters += n;
        length -= n;
    }

    return true;
}

void SimpleFontData::determinePitch()
{
    m_treatAsFixedPitch = platformData().isFixedPitch();
}

float SimpleFontData::platformWidthForGlyph(Glyph glyph) const
{
    SkASSERT(sizeof(glyph) == 2);   // compile-time assert

    SkPaint paint;

    m_platformData.setupPaint(&paint);

    paint.setTextEncoding(SkPaint::kGlyphID_TextEncoding);
    SkScalar width = paint.measureText(&glyph, 2);

    return SkScalarToFloat(width);
}

}  // namespace WebCore
