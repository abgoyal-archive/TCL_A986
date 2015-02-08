

#include "config.h"
#include "JSCSSValue.h"

#include "CSSPrimitiveValue.h"
#include "CSSValueList.h"
#include "JSCSSPrimitiveValue.h"
#include "JSCSSValueList.h"
#include "JSWebKitCSSTransformValue.h"
#include "WebKitCSSTransformValue.h"

#if ENABLE(SVG)
#include "JSSVGColor.h"
#include "JSSVGPaint.h"
#include "SVGColor.h"
#include "SVGPaint.h"
#endif

using namespace JSC;

namespace WebCore {

JSValue toJS(ExecState* exec, JSDOMGlobalObject* globalObject, CSSValue* value)
{
    if (!value)
        return jsNull();

    DOMObject* wrapper = getCachedDOMObjectWrapper(exec, value);

    if (wrapper)
        return wrapper;

    if (value->isWebKitCSSTransformValue())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, WebKitCSSTransformValue, value);
    else if (value->isValueList())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSValueList, value);
#if ENABLE(SVG)
    else if (value->isSVGPaint())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, SVGPaint, value);
    else if (value->isSVGColor())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, SVGColor, value);
#endif
    else if (value->isPrimitiveValue())
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSPrimitiveValue, value);
    else
        wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSValue, value);

    return wrapper;
}

} // namespace WebCore
