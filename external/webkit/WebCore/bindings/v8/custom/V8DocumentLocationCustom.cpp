

#include "config.h"
#include "V8Document.h"

#include "DOMWindow.h"
#include "Frame.h"
#include "V8Binding.h"
#include "V8Location.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8Document::locationAccessorGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    Document* document = V8Document::toNative(info.Holder());
    if (!document->frame())
        return v8::Null();

    DOMWindow* window = document->frame()->domWindow();
    return toV8(window->location());
}

void V8Document::locationAccessorSetter(v8::Local<v8::String> name, v8::Local<v8::Value> value, const v8::AccessorInfo& info)
{
    Document* document = V8Document::toNative(info.Holder());
    if (!document->frame())
        return;

    DOMWindow* window = document->frame()->domWindow();
    // setLocation does security checks. // XXXMB- verify!
    V8DOMWindowShell::setLocation(window, toWebCoreString(value));
}

} // namespace WebCore
