

#ifndef CSSCharsetRule_h
#define CSSCharsetRule_h

#include "CSSRule.h"
#include "PlatformString.h"

namespace WebCore {

class CSSCharsetRule : public CSSRule {
public:
    static PassRefPtr<CSSCharsetRule> create(CSSStyleSheet* parent, const String& encoding)
    {
        return adoptRef(new CSSCharsetRule(parent, encoding));
    }

    virtual ~CSSCharsetRule();

    const String& encoding() const { return m_encoding; }
    void setEncoding(const String& encoding, ExceptionCode&) { m_encoding = encoding; }

    virtual String cssText() const;

private:
    CSSCharsetRule(CSSStyleSheet* parent, const String& encoding);

    virtual bool isCharsetRule() { return true; }

    // from CSSRule
    virtual unsigned short type() const { return CHARSET_RULE; }

    String m_encoding;
};

} // namespace WebCore

#endif // CSSCharsetRule_h
