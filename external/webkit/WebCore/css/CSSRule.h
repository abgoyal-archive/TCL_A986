

#ifndef CSSRule_h
#define CSSRule_h

#include "CSSStyleSheet.h"
#include "KURLHash.h"
#include <wtf/ListHashSet.h>

namespace WebCore {

typedef int ExceptionCode;

class CSSRule : public StyleBase {
public:
    // FIXME: Change name to Type.
    enum CSSRuleType {
        UNKNOWN_RULE,
        STYLE_RULE,
        CHARSET_RULE,
        IMPORT_RULE,
        MEDIA_RULE,
        FONT_FACE_RULE,
        PAGE_RULE,
        VARIABLES_RULE,
        WEBKIT_KEYFRAMES_RULE,
        WEBKIT_KEYFRAME_RULE
    };

    // FIXME: Change to return CSSRuleType.
    virtual unsigned short type() const = 0;

    CSSStyleSheet* parentStyleSheet() const;
    CSSRule* parentRule() const;

    virtual String cssText() const = 0;
    void setCssText(const String&, ExceptionCode&);

    virtual void addSubresourceStyleURLs(ListHashSet<KURL>&) { }

protected:
    CSSRule(CSSStyleSheet* parent)
        : StyleBase(parent)
    {
    }

private:
    virtual bool isRule() { return true; }
};

} // namespace WebCore

#endif // CSSRule_h
