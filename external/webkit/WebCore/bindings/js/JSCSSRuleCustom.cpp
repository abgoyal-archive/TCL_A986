

#include "config.h"
#include "JSCSSRule.h"

#include "CSSCharsetRule.h"
#include "CSSFontFaceRule.h"
#include "CSSImportRule.h"
#include "CSSMediaRule.h"
#include "CSSPageRule.h"
#include "CSSStyleRule.h"
#include "CSSVariablesRule.h"
#include "JSCSSCharsetRule.h"
#include "JSCSSFontFaceRule.h"
#include "JSCSSImportRule.h"
#include "JSCSSMediaRule.h"
#include "JSCSSPageRule.h"
#include "JSCSSStyleRule.h"
#include "JSCSSVariablesRule.h"
#include "JSWebKitCSSKeyframeRule.h"
#include "JSWebKitCSSKeyframesRule.h"
#include "WebKitCSSKeyframeRule.h"
#include "WebKitCSSKeyframesRule.h"

using namespace JSC;

namespace WebCore {

JSValue toJS(ExecState* exec, JSDOMGlobalObject* globalObject, CSSRule* rule)
{
    if (!rule)
        return jsNull();

    DOMObject* wrapper = getCachedDOMObjectWrapper(exec, rule);
    if (wrapper)
        return wrapper;

    switch (rule->type()) {
        case CSSRule::STYLE_RULE:
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSStyleRule, rule);
            break;
        case CSSRule::MEDIA_RULE:
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSMediaRule, rule);
            break;
        case CSSRule::FONT_FACE_RULE:
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSFontFaceRule, rule);
            break;
        case CSSRule::PAGE_RULE:
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSPageRule, rule);
            break;
        case CSSRule::IMPORT_RULE:
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSImportRule, rule);
            break;
        case CSSRule::CHARSET_RULE:
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSCharsetRule, rule);
            break;
        case CSSRule::VARIABLES_RULE:
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSVariablesRule, rule);
            break;
        case CSSRule::WEBKIT_KEYFRAME_RULE:
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, WebKitCSSKeyframeRule, rule);
            break;
        case CSSRule::WEBKIT_KEYFRAMES_RULE:
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, WebKitCSSKeyframesRule, rule);
            break;
        default:
            wrapper = CREATE_DOM_OBJECT_WRAPPER(exec, globalObject, CSSRule, rule);
    }

    return wrapper;
}

} // namespace WebCore
