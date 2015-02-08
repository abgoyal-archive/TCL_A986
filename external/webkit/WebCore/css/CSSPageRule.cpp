

#include "config.h"
#include "CSSPageRule.h"

#include "CSSMutableStyleDeclaration.h"

namespace WebCore {

CSSPageRule::CSSPageRule(CSSStyleSheet* parent)
    : CSSRule(parent)
{
}

CSSPageRule::~CSSPageRule()
{
}

String CSSPageRule::selectorText() const
{
    // FIXME: Implement!
    return String();
}

void CSSPageRule::setSelectorText(const String& /*selectorText*/, ExceptionCode& /*ec*/)
{
    // FIXME: Implement!
}

String CSSPageRule::cssText() const
{
    // FIXME: Implement!
    return String();
}

} // namespace WebCore
