

#include "config.h"
#include "WebKitCSSKeyframeRule.h"

#include "CSSMutableStyleDeclaration.h"

namespace WebCore {

WebKitCSSKeyframeRule::WebKitCSSKeyframeRule(CSSStyleSheet* parent)
    : CSSRule(parent)
{
}

WebKitCSSKeyframeRule::~WebKitCSSKeyframeRule()
{
    if (m_style)
        m_style->setParent(0);
}

String WebKitCSSKeyframeRule::cssText() const
{
    String result = m_key;

    result += " { ";
    result += m_style->cssText();
    result += "}";

    return result;
}

bool WebKitCSSKeyframeRule::parseString(const String& /*string*/, bool /*strict*/)
{
    // FIXME
    return false;
}

void WebKitCSSKeyframeRule::setDeclaration(PassRefPtr<CSSMutableStyleDeclaration> style)
{
    m_style = style;
    m_style->setParent(parent());
}

/* static */
void WebKitCSSKeyframeRule::parseKeyString(const String& s, Vector<float>& keys)
{
    keys.clear();
    Vector<String> strings;
    s.split(',', strings);
    
    for (size_t i = 0; i < strings.size(); ++i) {
        float key = -1;
        String cur = strings[i].stripWhiteSpace();
    
        // For now the syntax MUST be 'xxx%' or 'from' or 'to', where xxx is a legal floating point number
        if (cur == "from")
            key = 0;
        else if (cur == "to")
            key = 1;
        else if (cur.endsWith("%")) {
            float k = cur.substring(0, cur.length() - 1).toFloat();
            if (k >= 0 && k <= 100)
                key = k/100;
        }
        
        if (key < 0) {
            keys.clear();
            return;
        }
        else
            keys.append(key);
    }
}

} // namespace WebCore
