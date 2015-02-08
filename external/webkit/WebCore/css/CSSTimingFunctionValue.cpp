

#include "config.h"
#include "CSSTimingFunctionValue.h"

#include "PlatformString.h"

namespace WebCore {

String CSSTimingFunctionValue::cssText() const
{
    String text("cubic-bezier(");
    text += String::number(m_x1);
    text += ", ";
    text += String::number(m_y1);
    text += ", ";
    text += String::number(m_x2);
    text += ", ";
    text += String::number(m_y2);
    text += ")";
    return text;
}

} // namespace WebCore
