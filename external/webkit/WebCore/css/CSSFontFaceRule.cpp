

#include "config.h"
#include "CSSFontFaceRule.h"

#include "CSSMutableStyleDeclaration.h"

namespace WebCore {

CSSFontFaceRule::CSSFontFaceRule(CSSStyleSheet* parent)
    : CSSRule(parent)
{
}

CSSFontFaceRule::~CSSFontFaceRule()
{
}

void CSSFontFaceRule::setDeclaration(PassRefPtr<CSSMutableStyleDeclaration> style)
{
    m_style = style;
}

String CSSFontFaceRule::cssText() const
{
    String result("@font-face");
    result += " { ";
    result += m_style->cssText();
    result += "}";
    return result;
}

void CSSFontFaceRule::addSubresourceStyleURLs(ListHashSet<KURL>& urls)
{
    if (m_style)
        m_style->addSubresourceStyleURLs(urls);
}

} // namespace WebCore
