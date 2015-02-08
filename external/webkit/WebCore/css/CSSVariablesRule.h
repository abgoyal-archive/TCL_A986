

#ifndef CSSVariablesRule_h
#define CSSVariablesRule_h

#include "CSSRule.h"
#include "CSSVariablesDeclaration.h"
#include <wtf/RefPtr.h>

namespace WebCore {

class CSSStyleSheet;
class MediaList;

class CSSVariablesRule : public CSSRule {
public:
    static PassRefPtr<CSSVariablesRule> create(CSSStyleSheet* parent, MediaList* mediaList, bool variablesKeyword)
    {
        return adoptRef(new CSSVariablesRule(parent, mediaList, variablesKeyword));
    }

    virtual ~CSSVariablesRule();

    // CSSVariablesRule interface
    MediaList* media() const { return m_lstMedia.get(); }
    CSSVariablesDeclaration* variables() { return m_variables.get(); }

    // Inherited from CSSRule
    virtual unsigned short type() const { return VARIABLES_RULE; }
    virtual String cssText() const;
    virtual bool isVariablesRule() { return true; }

    // Used internally.  Does not notify the document of the change.  Only intended
    // for use on initial parse.
    void setDeclaration(PassRefPtr<CSSVariablesDeclaration> decl) { m_variables = decl; }

private:
    CSSVariablesRule(CSSStyleSheet* parent, MediaList*, bool variablesKeyword);

    RefPtr<MediaList> m_lstMedia;
    RefPtr<CSSVariablesDeclaration> m_variables;
    bool m_variablesKeyword;
};

} // namespace WebCore

#endif // CSSVariablesRule_h
