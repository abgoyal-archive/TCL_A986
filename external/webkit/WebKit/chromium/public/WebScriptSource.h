

#ifndef WebScriptSource_h
#define WebScriptSource_h

#include "WebString.h"
#include "WebURL.h"

namespace WebKit {

struct WebScriptSource {
    WebString code;
    WebURL url;
    int startLine;

    WebScriptSource(const WebString& code)
        : code(code), startLine(1) { }
    WebScriptSource(const WebString& code, const WebURL& url)
        : code(code), url(url), startLine(1) { }
    WebScriptSource(const WebString& code, const WebURL& url, int startLine)
        : code(code), url(url), startLine(startLine) { }
};

} // namespace WebKit

#endif
