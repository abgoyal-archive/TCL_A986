

#ifndef WebDevToolsMessageData_h
#define WebDevToolsMessageData_h

#include "WebCommon.h"
#include "WebString.h"
#include "WebVector.h"

namespace WebKit {

struct WebDevToolsMessageData {
    WebString className;
    WebString methodName;
    WebVector<WebString> arguments;
};

} // namespace WebKit

#endif
