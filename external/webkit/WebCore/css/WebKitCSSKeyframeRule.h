

#ifndef WebKitCSSKeyframeRule_h
#define WebKitCSSKeyframeRule_h

#include "CSSRule.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class CSSMutableStyleDeclaration;

typedef int ExceptionCode;

class WebKitCSSKeyframeRule : public CSSRule {
public:
    static PassRefPtr<WebKitCSSKeyframeRule> create()
    {
        return adoptRef(new WebKitCSSKeyframeRule(0));
    }
    static PassRefPtr<WebKitCSSKeyframeRule> create(CSSStyleSheet* parent)
    {
        return adoptRef(new WebKitCSSKeyframeRule(parent));
    }

    virtual ~WebKitCSSKeyframeRule();

    virtual bool isKeyframeRule() { return true; }

    // Inherited from CSSRule
    virtual unsigned short type() const { return WEBKIT_KEYFRAME_RULE; }

    String keyText() const              { return m_key; }
    void setKeyText(const String& s)    { m_key = s; }
    
    void getKeys(Vector<float>& keys) const   { parseKeyString(m_key, keys); }

    CSSMutableStyleDeclaration* style() const { return m_style.get(); }

    virtual String cssText() const;

    // Not part of the CSSOM
    virtual bool parseString(const String&, bool = false);
    
    void setDeclaration(PassRefPtr<CSSMutableStyleDeclaration>);

    CSSMutableStyleDeclaration*         declaration()       { return m_style.get(); }
    const CSSMutableStyleDeclaration*   declaration() const { return m_style.get(); }
    
private:
    static void parseKeyString(const String& s, Vector<float>& keys);
    
    WebKitCSSKeyframeRule(CSSStyleSheet* parent);

    RefPtr<CSSMutableStyleDeclaration> m_style;
    String m_key;        // comma separated list of keys
};

} // namespace WebCore

#endif // WebKitCSSKeyframeRule_h
