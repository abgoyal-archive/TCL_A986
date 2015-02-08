

#ifndef CSSMediaRule_h
#define CSSMediaRule_h

#include "CSSRule.h"
#include "CSSRuleList.h"
#include "MediaList.h"
#include "PlatformString.h" // needed so bindings will compile

namespace WebCore {

class CSSRuleList;
class MediaList;

class CSSMediaRule : public CSSRule {
public:
    static PassRefPtr<CSSMediaRule> create(CSSStyleSheet* parent, PassRefPtr<MediaList> media, PassRefPtr<CSSRuleList> rules)
    {
        return adoptRef(new CSSMediaRule(parent, media, rules));
    }
    virtual ~CSSMediaRule();

    MediaList* media() const { return m_lstMedia.get(); }
    CSSRuleList* cssRules() { return m_lstCSSRules.get(); }

    unsigned insertRule(const String& rule, unsigned index, ExceptionCode&);
    void deleteRule(unsigned index, ExceptionCode&);

    virtual String cssText() const;

    // Not part of the CSSOM
    unsigned append(CSSRule*);

private:
    CSSMediaRule(CSSStyleSheet* parent, PassRefPtr<MediaList>, PassRefPtr<CSSRuleList>);

    virtual bool isMediaRule() { return true; }

    // Inherited from CSSRule
    virtual unsigned short type() const { return MEDIA_RULE; }

    RefPtr<MediaList> m_lstMedia;
    RefPtr<CSSRuleList> m_lstCSSRules;
};

} // namespace WebCore

#endif // CSSMediaRule_h
