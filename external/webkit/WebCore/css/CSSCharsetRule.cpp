

#include "config.h"
#include "CSSCharsetRule.h"

namespace WebCore {

CSSCharsetRule::CSSCharsetRule(CSSStyleSheet* parent, const String& encoding)
    : CSSRule(parent)
    , m_encoding(encoding)
{
}

CSSCharsetRule::~CSSCharsetRule()
{
}

String CSSCharsetRule::cssText() const
{
    return "@charset \"" + m_encoding + "\";";
}

} // namespace WebCore
