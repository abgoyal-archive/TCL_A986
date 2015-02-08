
#include "config.h"
#include "ShadowValue.h"

#include "CSSPrimitiveValue.h"
#include "PlatformString.h"

namespace WebCore {

// Used for text-shadow and box-shadow
ShadowValue::ShadowValue(PassRefPtr<CSSPrimitiveValue> _x,
    PassRefPtr<CSSPrimitiveValue> _y,
    PassRefPtr<CSSPrimitiveValue> _blur,
    PassRefPtr<CSSPrimitiveValue> _spread,
    PassRefPtr<CSSPrimitiveValue> _style,
    PassRefPtr<CSSPrimitiveValue> _color)
    : x(_x)
    , y(_y)
    , blur(_blur)
    , spread(_spread)
    , style(_style)
    , color(_color)
{
}

String ShadowValue::cssText() const
{
    String text("");

    if (color)
        text += color->cssText();
    if (x) {
        if (!text.isEmpty())
            text += " ";
        text += x->cssText();
    }
    if (y) {
        if (!text.isEmpty())
            text += " ";
        text += y->cssText();
    }
    if (blur) {
        if (!text.isEmpty())
            text += " ";
        text += blur->cssText();
    }
    if (spread) {
        if (!text.isEmpty())
            text += " ";
        text += spread->cssText();
    }
    if (style) {
        if (!text.isEmpty())
            text += " ";
        text += style->cssText();
    }

    return text;
}

}
