
#include "config.h"
#include "FontValue.h"

#include "CSSValueList.h"
#include "CSSPrimitiveValue.h"
#include "PlatformString.h"

namespace WebCore {

String FontValue::cssText() const
{
    // font variant weight size / line-height family 

    String result("");

    if (style)
        result += style->cssText();
    if (variant) {
        if (!result.isEmpty())
            result += " ";
        result += variant->cssText();
    }
    if (weight) {
        if (!result.isEmpty())
            result += " ";
        result += weight->cssText();
    }
    if (size) {
        if (!result.isEmpty())
            result += " ";
        result += size->cssText();
    }
    if (lineHeight) {
        if (!size)
            result += " ";
        result += "/";
        result += lineHeight->cssText();
    }
    if (family) {
        if (!result.isEmpty())
            result += " ";
        result += family->cssText();
    }

    return result;
}

}
