

#ifndef WebRegularExpression_h
#define WebRegularExpression_h

#include "WebCommon.h"
#include "WebNonCopyable.h"
#include "WebTextCaseSensitivity.h"

namespace WebKit {

class WebRegularExpressionPrivate;
class WebString;

class WebRegularExpression : public WebNonCopyable {
public:
    WEBKIT_API WebRegularExpression(const WebString& pattern, WebTextCaseSensitivity);
    WEBKIT_API ~WebRegularExpression();

    WEBKIT_API int match(const WebString&, int startFrom = 0, int* matchLength = 0) const;

private:
    WebRegularExpressionPrivate* m_private;
};

} // namespace WebKit

#endif
