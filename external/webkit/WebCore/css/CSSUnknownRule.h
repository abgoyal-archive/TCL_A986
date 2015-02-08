

#ifndef CSSUnknownRule_h
#define CSSUnknownRule_h

#include "CSSRule.h"

namespace WebCore {

class CSSUnknownRule : public CSSRule {
private:
    virtual unsigned short type() const { return UNKNOWN_RULE; }
};

} // namespace WebCore

#endif // CSSUnknownRule_h
