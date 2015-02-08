

#include "config.h"
#include "V8HTMLIFrameElement.h"

#include "HTMLIFrameElement.h"
#include "HTMLNames.h" 
#include "V8Binding.h"
#include "V8BindingState.h"
#include "V8Proxy.h"

namespace WebCore {

using namespace HTMLNames; 

void V8HTMLIFrameElement::srcAccessorSetter(v8::Local<v8::String> name, v8::Local<v8::Value> value, const v8::AccessorInfo& info)
{
    HTMLIFrameElement* iframe = V8HTMLIFrameElement::toNative(info.Holder());
    String v = toWebCoreStringWithNullCheck(value);

    if (!V8BindingSecurity::allowSettingFrameSrcToJavascriptUrl(V8BindingState::Only(), iframe, v))
        return;

    iframe->setAttribute(srcAttr, v); 
}

} // namespace WebCore
