

#include "config.h"

#include "EmojiFont.h"
#include "Font.h"
#include "FontCache.h"
#include "SimpleFontData.h"
#include "FloatRect.h"
#include "FontDescription.h"
#include "SkPaint.h"
#include "SkTypeface.h"
#include "SkTime.h"

using namespace android;

namespace WebCore {

void SimpleFontData::platformInit()
{
    SkPaint  paint;
    SkPaint::FontMetrics metrics;
    
    m_platformData.setupPaint(&paint);
    (void)paint.getFontMetrics(&metrics);

    // use ceil instead of round to favor descent, given a lot of accidental
    // clipping of descenders (e.g. 14pt 'g') in textedit fields
    int d = SkScalarCeil(metrics.fDescent);
    int s = SkScalarRound(metrics.fDescent - metrics.fAscent);
    int a = s - d;

    m_ascent = a;
    m_descent = d;
    m_xHeight = SkScalarToFloat(-metrics.fAscent) * 0.56f;   // hack I stole from the window's port
    m_lineSpacing = a + d;
    m_lineGap = SkScalarRound(metrics.fLeading);
}

void SimpleFontData::platformCharWidthInit()
{
    m_avgCharWidth = 0.f;
    m_maxCharWidth = 0.f;
    initCharWidths();
}

void SimpleFontData::platformDestroy()
{
    delete m_smallCapsFontData;
}

SimpleFontData* SimpleFontData::smallCapsFontData(const FontDescription& fontDescription) const
{
    if (!m_smallCapsFontData) {
        m_smallCapsFontData = new SimpleFontData(FontPlatformData(m_platformData, fontDescription.computedSize() * 0.7f));
    }
    return m_smallCapsFontData;
}

bool SimpleFontData::containsCharacters(const UChar* characters, int length) const
{
    SkPaint     paint;

    m_platformData.setupPaint(&paint);
    paint.setTextEncoding(SkPaint::kUTF16_TextEncoding);
    return paint.containsText(characters, length << 1);
}

void SimpleFontData::determinePitch()
{
    m_treatAsFixedPitch = false;
}

float SimpleFontData::platformWidthForGlyph(Glyph glyph) const
{
    SkASSERT(sizeof(glyph) == 2);   // compile-time assert

    SkPaint  paint;

    m_platformData.setupPaint(&paint);

    if (EmojiFont::IsEmojiGlyph(glyph))
        return EmojiFont::GetAdvanceWidth(glyph, paint);
    else {
        paint.setTextEncoding(SkPaint::kGlyphID_TextEncoding);
        return SkScalarToFloat(paint.measureText(&glyph, 2));
    }
}

}
