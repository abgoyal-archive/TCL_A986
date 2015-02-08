

#include "config.h"
#include "V8NodeFilter.h"

#include "ExceptionCode.h"
#include "NodeFilter.h"

#include "V8Binding.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8NodeFilter::acceptNodeCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.NodeFilter.acceptNode()");
    return throwError(NOT_SUPPORTED_ERR);
}

} // namespace WebCore
