

#include "config.h"
#include "FontDescription.h"

namespace WebCore {

FontWeight FontDescription::lighterWeight(void) const
{
    // FIXME: Should actually return the CSS weight corresponding to next lightest
    // weight of the currently used font family.
    switch (m_weight) {
        case FontWeight100:
        case FontWeight200:
            return FontWeight100;

        case FontWeight300:
            return FontWeight200;

        case FontWeight400:
        case FontWeight500:
            return FontWeight300;

        case FontWeight600:
        case FontWeight700:
            return FontWeight400;

        case FontWeight800:
            return FontWeight500;

        case FontWeight900:
            return FontWeight700;
    }
    ASSERT_NOT_REACHED();
    return FontWeightNormal;
}

FontWeight FontDescription::bolderWeight(void) const
{
    // FIXME: Should actually return the CSS weight corresponding to next heaviest
    // weight of the currently used font family.
    switch (m_weight) {
        case FontWeight100:
        case FontWeight200:
            return FontWeight300;

        case FontWeight300:
            return FontWeight400;

        case FontWeight400:
        case FontWeight500:
            return FontWeight700;

        case FontWeight600:
        case FontWeight700:
            return FontWeight800;

        case FontWeight800:
        case FontWeight900:
            return FontWeight900;
    }
    ASSERT_NOT_REACHED();
    return FontWeightNormal;
}

FontTraitsMask FontDescription::traitsMask() const
{
    return static_cast<FontTraitsMask>((m_italic ? FontStyleItalicMask : FontStyleNormalMask)
            | (m_smallCaps ? FontVariantSmallCapsMask : FontVariantNormalMask)
            | (FontWeight100Mask << (m_weight - FontWeight100)));
    
}

} // namespace WebCore
