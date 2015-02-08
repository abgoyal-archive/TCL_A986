

#ifndef CSSPageRule_h
#define CSSPageRule_h

#include "CSSRule.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class CSSMutableStyleDeclaration;

class CSSPageRule : public CSSRule {
public:
    static PassRefPtr<CSSPageRule> create(CSSStyleSheet* parent)
    {
        return adoptRef(new CSSPageRule(parent));
    }

    virtual ~CSSPageRule();

    String selectorText() const;
    void setSelectorText(const String&, ExceptionCode&);

    CSSMutableStyleDeclaration* style() const { return m_style.get(); }

    virtual String cssText() const;

private:
    CSSPageRule(CSSStyleSheet* parent);

    // Inherited from CSSRule
    virtual unsigned short type() const { return PAGE_RULE; }

    RefPtr<CSSMutableStyleDeclaration> m_style;
};

} // namespace WebCore

#endif // CSSPageRule_h
