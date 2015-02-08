

#include "config.h"
#include "CSSInitialValue.h"

#include "PlatformString.h"

namespace WebCore {

unsigned short CSSInitialValue::cssValueType() const
{ 
    return CSS_INITIAL; 
}

String CSSInitialValue::cssText() const
{
    return "initial";
}

} // namespace WebCore
