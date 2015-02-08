

#include "config.h"
#include "V8CSSValue.h"

#include "V8CSSPrimitiveValue.h"
#include "V8CSSValueList.h"
#include "V8SVGColor.h"
#include "V8SVGPaint.h"
#include "V8WebKitCSSTransformValue.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(CSSValue* impl)
{
    if (!impl)
        return v8::Null();
    if (impl->isWebKitCSSTransformValue())
        return toV8(static_cast<WebKitCSSTransformValue*>(impl));
    if (impl->isValueList())
        return toV8(static_cast<CSSValueList*>(impl));
    if (impl->isPrimitiveValue())
        return toV8(static_cast<CSSPrimitiveValue*>(impl));
#if ENABLE(SVG)
    if (impl->isSVGPaint())
        return toV8(static_cast<SVGPaint*>(impl));
    if (impl->isSVGColor())
        return toV8(static_cast<SVGColor*>(impl));
#endif
    return V8CSSValue::wrap(impl);
}

} // namespace WebCore
