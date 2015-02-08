

#ifndef CSSFontFaceRule_h
#define CSSFontFaceRule_h

#include "CSSRule.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class CSSMutableStyleDeclaration;

class CSSFontFaceRule : public CSSRule {
public:
    static PassRefPtr<CSSFontFaceRule> create()
    {
        return adoptRef(new CSSFontFaceRule(0));
    }
    static PassRefPtr<CSSFontFaceRule> create(CSSStyleSheet* parent)
    {
        return adoptRef(new CSSFontFaceRule(parent));
    }

    virtual ~CSSFontFaceRule();

    CSSMutableStyleDeclaration* style() const { return m_style.get(); }

    virtual String cssText() const;

    void setDeclaration(PassRefPtr<CSSMutableStyleDeclaration>);

    virtual void addSubresourceStyleURLs(ListHashSet<KURL>& urls);

private:
    CSSFontFaceRule(CSSStyleSheet* parent);

    virtual bool isFontFaceRule() { return true; }

    // Inherited from CSSRule
    virtual unsigned short type() const { return FONT_FACE_RULE; }

    RefPtr<CSSMutableStyleDeclaration> m_style;
};

} // namespace WebCore

#endif // CSSFontFaceRule_h
