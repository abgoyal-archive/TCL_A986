

#include "config.h"
#include "CSSReflectValue.h"

#include "CSSPrimitiveValue.h"
#include "PlatformString.h"

using namespace std;

namespace WebCore {

String CSSReflectValue::cssText() const
{
    String result;
    switch (m_direction) {
        case ReflectionBelow:
            result += "below ";
            break;
        case ReflectionAbove:
            result += "above ";
            break;
        case ReflectionLeft:
            result += "left ";
            break;
        case ReflectionRight:
            result += "right ";
            break;
        default:
            break;
    }
    
    result += m_offset->cssText() + " ";
    if (m_mask)
        result += m_mask->cssText();
    return result;
}

void CSSReflectValue::addSubresourceStyleURLs(ListHashSet<KURL>& urls, const CSSStyleSheet* styleSheet)
{
    if (m_mask)
        m_mask->addSubresourceStyleURLs(urls, styleSheet);
}

} // namespace WebCore
