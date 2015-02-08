

#include "config.h"
#include "WebRegularExpression.h"

#include "RegularExpression.h"
#include "WebString.h"

using namespace WebCore;

namespace WebKit {

class WebRegularExpressionPrivate : public RegularExpression {
};

WebRegularExpression::WebRegularExpression(const WebString& pattern,
                                           WebTextCaseSensitivity caseSensitivity)
{
    TextCaseSensitivity sensitivity = static_cast<TextCaseSensitivity>(caseSensitivity);
    RegularExpression* re = new RegularExpression(pattern, sensitivity);
    m_private = static_cast<WebRegularExpressionPrivate*>(re);
}

WebRegularExpression::~WebRegularExpression()
{
    delete m_private;
}

int WebRegularExpression::match(const WebString& str,
                                int startFrom,
                                int* matchLength) const
{
    if (!m_private)
        return -1;

    return m_private->match(str, startFrom, matchLength);
}

} // namespace WebKit
