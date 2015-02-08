

#ifndef CSSRuleList_h
#define CSSRuleList_h

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class CSSRule;
class StyleList;

class CSSRuleList : public RefCounted<CSSRuleList> {
public:
    static PassRefPtr<CSSRuleList> create(StyleList* list, bool omitCharsetRules = false)
    {
        return adoptRef(new CSSRuleList(list, omitCharsetRules));
    }
    static PassRefPtr<CSSRuleList> create()
    {
        return adoptRef(new CSSRuleList);
    }
    ~CSSRuleList();

    unsigned length() const;
    CSSRule* item(unsigned index);

    // FIXME: Not part of the DOM.  Only used by media rules.  We should be able to remove them if we changed media rules to work
    // as StyleLists instead.
    unsigned insertRule(CSSRule*, unsigned index);
    void deleteRule(unsigned index);
    void append(CSSRule*);

private:
    CSSRuleList();
    CSSRuleList(StyleList*, bool omitCharsetRules);

    RefPtr<StyleList> m_list;
    Vector<RefPtr<CSSRule> > m_lstCSSRules; // FIXME: Want to eliminate, but used by IE rules() extension and still used by media rules.
};

} // namespace WebCore

#endif // CSSRuleList_h
