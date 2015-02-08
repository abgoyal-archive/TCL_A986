

#include "config.h"
#include "V8CSSRule.h"

#include "V8CSSCharsetRule.h"
#include "V8CSSFontFaceRule.h"
#include "V8CSSImportRule.h"
#include "V8CSSMediaRule.h"
#include "V8CSSPageRule.h"
#include "V8CSSStyleRule.h"
#include "V8CSSVariablesRule.h"
#include "V8WebKitCSSKeyframeRule.h"
#include "V8WebKitCSSKeyframesRule.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(CSSRule* impl)
{
    if (!impl)
        return v8::Null();
    switch (impl->type()) {
    case CSSRule::STYLE_RULE:
        return toV8(static_cast<CSSStyleRule*>(impl));
    case CSSRule::CHARSET_RULE:
        return toV8(static_cast<CSSCharsetRule*>(impl));
    case CSSRule::IMPORT_RULE:
        return toV8(static_cast<CSSImportRule*>(impl));
    case CSSRule::MEDIA_RULE:
        return toV8(static_cast<CSSMediaRule*>(impl));
    case CSSRule::FONT_FACE_RULE:
        return toV8(static_cast<CSSFontFaceRule*>(impl));
    case CSSRule::PAGE_RULE:
        return toV8(static_cast<CSSPageRule*>(impl));
    case CSSRule::VARIABLES_RULE:
        return toV8(static_cast<CSSVariablesRule*>(impl));
    case CSSRule::WEBKIT_KEYFRAME_RULE:
        return toV8(static_cast<WebKitCSSKeyframeRule*>(impl));
    case CSSRule::WEBKIT_KEYFRAMES_RULE:
        return toV8(static_cast<WebKitCSSKeyframesRule*>(impl));
    }
    return V8CSSRule::wrap(impl);
}

} // namespace WebCore
