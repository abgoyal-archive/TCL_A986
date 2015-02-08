

#include "config.h"
#include "CSSRule.h"

#include "NotImplemented.h"

namespace WebCore {

CSSStyleSheet* CSSRule::parentStyleSheet() const
{
    StyleBase* curr = parent();
    while (curr && !curr->isCSSStyleSheet())
        curr = curr->parent();
    return curr ? static_cast<CSSStyleSheet*>(curr) : 0;
}

CSSRule* CSSRule::parentRule() const
{
    return (parent() && parent()->isRule()) ? static_cast<CSSRule*>(parent()) : 0;
}

void CSSRule::setCssText(const String& /*cssText*/, ExceptionCode& /*ec*/)
{
    notImplemented();
}

} // namespace WebCore
