

#include "config.h"
#include "CSSInheritedValue.h"

#include "PlatformString.h"

namespace WebCore {

unsigned short CSSInheritedValue::cssValueType() const
{
    return CSS_INHERIT;
}

String CSSInheritedValue::cssText() const
{
    return "inherit";
}

} // namespace WebCore
