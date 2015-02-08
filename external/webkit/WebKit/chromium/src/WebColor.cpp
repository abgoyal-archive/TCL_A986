

#include "config.h"
#include "WebColor.h"

#include "Color.h"
#include "CSSValueKeywords.h"
#include "RenderTheme.h"
#include "UnusedParam.h"
#include "WebColorName.h"

using namespace::WebCore;

namespace WebKit {

static int toCSSValueKeyword(WebColorName in_value)
{
    switch (in_value) {
    case WebColorActiveBorder:
        return CSSValueActiveborder;
    case WebColorActiveCaption:
        return CSSValueActivecaption;
    case WebColorAppworkspace:
        return CSSValueAppworkspace;
    case WebColorBackground:
        return CSSValueBackground;
    case WebColorButtonFace:
        return CSSValueButtonface;
    case WebColorButtonHighlight:
        return CSSValueButtonhighlight;
    case WebColorButtonShadow:
        return CSSValueButtonshadow;
    case WebColorButtonText:
        return CSSValueButtontext;
    case WebColorCaptionText:
        return CSSValueCaptiontext;
    case WebColorGrayText:
        return CSSValueGraytext;
    case WebColorHighlight:
        return CSSValueHighlight;
    case WebColorHighlightText:
        return CSSValueHighlighttext;
    case WebColorInactiveBorder:
        return CSSValueInactiveborder;
    case WebColorInactiveCaption:
        return CSSValueInactivecaption;
    case WebColorInactiveCaptionText:
        return CSSValueInactivecaptiontext;
    case WebColorInfoBackground:
        return CSSValueInfobackground;
    case WebColorInfoText:
        return CSSValueInfotext;
    case WebColorMenu:
        return CSSValueMenu;
    case WebColorMenuText:
        return CSSValueMenutext;
    case WebColorScrollbar:
        return CSSValueScrollbar;
    case WebColorText:
        return CSSValueText;
    case WebColorThreedDarkShadow:
        return CSSValueThreeddarkshadow;
    case WebColorThreedShadow:
        return CSSValueThreedshadow;
    case WebColorThreedFace:
        return CSSValueThreedface;
    case WebColorThreedHighlight:
        return CSSValueThreedhighlight;
    case WebColorThreedLightShadow:
        return CSSValueThreedlightshadow;
    case WebColorWebkitFocusRingColor:
        return CSSValueWebkitFocusRingColor;
    case WebColorWindow:
        return CSSValueWindow;
    case WebColorWindowFrame:
        return CSSValueWindowframe;
    case WebColorWindowText:
        return CSSValueWindowtext;
    default:
        return CSSValueInvalid;
    }
}

void setNamedColors(const WebColorName* colorNames, const WebColor* colors, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        WebColorName colorName = colorNames[i];
        WebColor color = colors[i];

        // Convert color to internal value identifier.
        int internalColorName = toCSSValueKeyword(colorName);
        if (internalColorName == CSSValueWebkitFocusRingColor) {
            RenderTheme::setCustomFocusRingColor(color);
            continue;
        }
    }

    // TODO(jeremy): Tell RenderTheme to update colors.
}

} // WebKit
