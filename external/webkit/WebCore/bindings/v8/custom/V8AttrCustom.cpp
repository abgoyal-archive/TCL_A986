

#include "config.h"
#include "V8Attr.h"

#include "Attr.h"
#include "Element.h"
#include "ExceptionCode.h"
#include "V8Binding.h"
#include "V8BindingState.h"
#include "V8Proxy.h"

namespace WebCore {

void V8Attr::valueAccessorSetter(v8::Local<v8::String> name, v8::Local<v8::Value> value, const v8::AccessorInfo& info)
{
    Attr* imp = V8Attr::toNative(info.Holder());
    String attrValue = toWebCoreStringWithNullCheck(value);
    Element* ownerElement = imp->ownerElement();

    if (ownerElement && !V8BindingSecurity::allowSettingSrcToJavascriptURL(V8BindingState::Only(), ownerElement, imp->name(), attrValue))
        return;

    ExceptionCode ec = 0;
    imp->setValue(attrValue, ec);
    if (ec)
        throwError(ec);
}

} // namespace WebCore
