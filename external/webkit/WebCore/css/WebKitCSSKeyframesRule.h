

#ifndef WebKitCSSKeyframesRule_h
#define WebKitCSSKeyframesRule_h

#include "CSSRule.h"
#include <wtf/RefPtr.h>
#include "AtomicString.h"

namespace WebCore {

class CSSRuleList;
class WebKitCSSKeyframeRule;
class String;

typedef int ExceptionCode;

class WebKitCSSKeyframesRule : public CSSRule {
public:
    static PassRefPtr<WebKitCSSKeyframesRule> create()
    {
        return adoptRef(new WebKitCSSKeyframesRule(0));
    }
    static PassRefPtr<WebKitCSSKeyframesRule> create(CSSStyleSheet* parent)
    {
        return adoptRef(new WebKitCSSKeyframesRule(parent));
    }

    ~WebKitCSSKeyframesRule();

    virtual bool isKeyframesRule() { return true; }

    // Inherited from CSSRule
    virtual unsigned short type() const { return WEBKIT_KEYFRAMES_RULE; }

    String name() const;
    void setName(const String&);
    
    // This version of setName does not call styleSheetChanged to avoid
    // unnecessary work. It assumes callers will either make that call
    // themselves, or know that it will get called later.
    void setNameInternal(const String& name)
    {   
        m_name = AtomicString(name);
    }

    CSSRuleList* cssRules() { return m_lstCSSRules.get(); }

    void insertRule(const String& rule);
    void deleteRule(const String& key);
    WebKitCSSKeyframeRule* findRule(const String& key);

    virtual String cssText() const;

    /* not part of the DOM */
    unsigned length() const;
    WebKitCSSKeyframeRule*        item(unsigned index);
    const WebKitCSSKeyframeRule*  item(unsigned index) const;
    void append(WebKitCSSKeyframeRule* rule);

private:
    WebKitCSSKeyframesRule(CSSStyleSheet* parent);

    int findRuleIndex(const String& key) const;
    
    RefPtr<CSSRuleList> m_lstCSSRules;
    AtomicString m_name;
};

} // namespace WebCore

#endif // WebKitCSSKeyframesRule_h
