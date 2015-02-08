

#include "config.h"
#include "V8HTMLFrameElement.h"

#include "HTMLFrameElement.h"
#include "HTMLNames.h"
#include "V8Binding.h"
#include "V8BindingState.h"
#include "V8Proxy.h"

namespace WebCore {

using namespace HTMLNames; 

void V8HTMLFrameElement::srcAccessorSetter(v8::Local<v8::String> name, v8::Local<v8::Value> value, const v8::AccessorInfo& info)
{
    HTMLFrameElement* frame = V8HTMLFrameElement::toNative(info.Holder());
    String srcValue = toWebCoreStringWithNullCheck(value);

    if (!V8BindingSecurity::allowSettingFrameSrcToJavascriptUrl(V8BindingState::Only(), frame, srcValue))
        return;

    frame->setAttribute(srcAttr, srcValue); 
}

void V8HTMLFrameElement::locationAccessorSetter(v8::Local<v8::String> name, v8::Local<v8::Value> value, const v8::AccessorInfo& info)
{
    HTMLFrameElement* frame = V8HTMLFrameElement::toNative(info.Holder());
    String locationValue = toWebCoreStringWithNullCheck(value);

    if (!V8BindingSecurity::allowSettingFrameSrcToJavascriptUrl(V8BindingState::Only(), frame, locationValue))
        return;

    frame->setLocation(locationValue);
}

} // namespace WebCore
