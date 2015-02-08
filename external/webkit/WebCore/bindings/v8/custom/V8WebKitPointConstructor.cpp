

#include "config.h"
#include "V8WebKitPoint.h"

#include "V8Binding.h"
#include "V8DOMWrapper.h"
#include "V8Index.h"
#include "V8Proxy.h"

#include <wtf/MathExtras.h>

namespace WebCore {

v8::Handle<v8::Value> V8WebKitPoint::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebKitPoint.Constructor");

    if (!args.IsConstructCall())
        return throwError("DOM object constructor cannot be called as a function.");

    float x = 0;
    float y = 0;
    if (args.Length() > 1) {
        if (!args[0]->IsUndefined()) {
            x = toFloat(args[0]);
            if (isnan(x))
                x = 0;
        }
        if (!args[1]->IsUndefined()) {
            y = toFloat(args[1]);
            if (isnan(y))
                y = 0;
        }
    }
    PassRefPtr<WebKitPoint> point = WebKitPoint::create(x, y);
    point->ref();
    V8DOMWrapper::setDOMWrapper(args.Holder(), V8ClassIndex::WEBKITPOINT, point.get());
    return args.Holder();
}

} // namespace WebCore
