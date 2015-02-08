

#include "config.h"
#include "UniscribeHelperTextRun.h"

#include "ChromiumBridge.h"
#include "Font.h"
#include "SimpleFontData.h"

namespace WebCore {

UniscribeHelperTextRun::UniscribeHelperTextRun(const TextRun& run,
                                               const Font& font)
    : UniscribeHelper(run.characters(), run.length(), run.rtl(),
                      font.primaryFont()->platformData().hfont(),
                      font.primaryFont()->platformData().scriptCache(),
                      font.primaryFont()->platformData().scriptFontProperties())
    , m_font(&font)
    , m_fontIndex(0)
{
    setDirectionalOverride(run.directionalOverride());
    setLetterSpacing(font.letterSpacing());
    setSpaceWidth(font.spaceWidth());
    setWordSpacing(font.wordSpacing());
    setAscent(font.primaryFont()->ascent());

    init();

    // Padding is the amount to add to make justification happen. This
    // should be done after Init() so all the runs are already measured.
    if (run.padding() > 0)
        justify(run.padding());
}

UniscribeHelperTextRun::UniscribeHelperTextRun(
    const wchar_t* input,
    int inputLength,
    bool isRtl,
    HFONT hfont,
    SCRIPT_CACHE* scriptCache,
    SCRIPT_FONTPROPERTIES* fontProperties)
    : UniscribeHelper(input, inputLength, isRtl, hfont,
                      scriptCache, fontProperties)
    , m_font(0)
    , m_fontIndex(-1)
{
}

void UniscribeHelperTextRun::tryToPreloadFont(HFONT font)
{
    // Ask the browser to get the font metrics for this font.
    // That will preload the font and it should now be accessible
    // from the renderer.
    ChromiumBridge::ensureFontLoaded(font);
}

bool UniscribeHelperTextRun::nextWinFontData(
    HFONT* hfont,
    SCRIPT_CACHE** scriptCache,
    SCRIPT_FONTPROPERTIES** fontProperties,
    int* ascent)
{
    // This check is necessary because NextWinFontData can be called again
    // after we already ran out of fonts. fontDataAt behaves in a strange
    // manner when the difference between param passed and # of fonts stored in
    // WebKit::Font is larger than one. We can avoid this check by setting
    // font_index_ to # of elements in hfonts_ when we run out of font. In that
    // case, we'd have to go through a couple of more checks before returning
    // false.
    if (m_fontIndex == -1 || !m_font)
        return false;

    // If the font data for a fallback font requested is not yet retrieved, add
    // them to our vectors. Note that '>' rather than '>=' is used to test that
    // condition. primaryFont is not stored in hfonts_, and friends so that
    // indices for fontDataAt and our vectors for font data are 1 off from each
    // other.  That is, when fully populated, hfonts_ and friends have one font
    // fewer than what's contained in font_.
    if (static_cast<size_t>(++m_fontIndex) > m_hfonts.size()) {
        const FontData *fontData = m_font->fontDataAt(m_fontIndex);
        if (!fontData) {
            // Ran out of fonts.
            m_fontIndex = -1;
            return false;
        }

        // FIXME: this won't work for SegmentedFontData
        // http://crbug.com/6425
        const SimpleFontData* simpleFontData =
            fontData->fontDataForCharacter(' ');

        m_hfonts.append(simpleFontData->platformData().hfont());
        m_scriptCaches.append(simpleFontData->platformData().scriptCache());
        m_fontProperties.append(simpleFontData->platformData().scriptFontProperties());
        m_ascents.append(simpleFontData->ascent());
    }

    *hfont = m_hfonts[m_fontIndex - 1];
    *scriptCache = m_scriptCaches[m_fontIndex - 1];
    *fontProperties = m_fontProperties[m_fontIndex - 1];
    *ascent = m_ascents[m_fontIndex - 1];
    return true;
}

void UniscribeHelperTextRun::resetFontIndex()
{
    m_fontIndex = 0;
}

}  // namespace WebCore
