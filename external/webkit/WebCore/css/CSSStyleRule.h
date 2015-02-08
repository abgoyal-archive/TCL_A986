

#ifndef CSSStyleRule_h
#define CSSStyleRule_h

#include "CSSRule.h"
#include "CSSSelectorList.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class CSSMutableStyleDeclaration;
class CSSSelector;

class CSSStyleRule : public CSSRule {
public:
    static PassRefPtr<CSSStyleRule> create(CSSStyleSheet* parent)
    {
        return adoptRef(new CSSStyleRule(parent));
    }
    virtual ~CSSStyleRule();

    String selectorText() const;
    void setSelectorText(const String&, ExceptionCode&);

    CSSMutableStyleDeclaration* style() const { return m_style.get(); }

    virtual String cssText() const;

    // Not part of the CSSOM
    virtual bool parseString(const String&, bool = false);

    void adoptSelectorVector(Vector<CSSSelector*>& selectors) { m_selectorList.adoptSelectorVector(selectors); }
    void setDeclaration(PassRefPtr<CSSMutableStyleDeclaration>);

    const CSSSelectorList& selectorList() const { return m_selectorList; }
    CSSMutableStyleDeclaration* declaration() { return m_style.get(); }

    virtual void addSubresourceStyleURLs(ListHashSet<KURL>& urls);

private:
    CSSStyleRule(CSSStyleSheet* parent);

    virtual bool isStyleRule() { return true; }

    // Inherited from CSSRule
    virtual unsigned short type() const { return STYLE_RULE; }

    RefPtr<CSSMutableStyleDeclaration> m_style;
    CSSSelectorList m_selectorList;
};

} // namespace WebCore

#endif // CSSStyleRule_h
