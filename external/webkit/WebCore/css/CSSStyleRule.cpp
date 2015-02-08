

#include "config.h"
#include "CSSStyleRule.h"

#include "CSSMutableStyleDeclaration.h"
#include "CSSSelector.h"

namespace WebCore {

CSSStyleRule::CSSStyleRule(CSSStyleSheet* parent)
    : CSSRule(parent)
{
}

CSSStyleRule::~CSSStyleRule()
{
    if (m_style)
        m_style->setParent(0);
}

String CSSStyleRule::selectorText() const
{
    String str;
    for (CSSSelector* s = selectorList().first(); s; s = CSSSelectorList::next(s)) {
        if (s != selectorList().first())
            str += ", ";
        str += s->selectorText();
    }
    return str;
}

void CSSStyleRule::setSelectorText(const String& /*selectorText*/, ExceptionCode& /*ec*/)
{
    // FIXME: Implement!
}

String CSSStyleRule::cssText() const
{
    String result = selectorText();

    result += " { ";
    result += m_style->cssText();
    result += "}";

    return result;
}

bool CSSStyleRule::parseString(const String& /*string*/, bool /*strict*/)
{
    // FIXME
    return false;
}

void CSSStyleRule::setDeclaration(PassRefPtr<CSSMutableStyleDeclaration> style)
{
    m_style = style;
}

void CSSStyleRule::addSubresourceStyleURLs(ListHashSet<KURL>& urls)
{
    if (m_style)
        m_style->addSubresourceStyleURLs(urls);
}

} // namespace WebCore
