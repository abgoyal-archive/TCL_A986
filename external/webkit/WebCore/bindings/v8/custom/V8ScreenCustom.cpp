

#include "config.h"
#include "V8Screen.h"

#include "V8DOMWindow.h"
#include "V8DOMWrapper.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(Screen* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = getDOMObjectMap().get(impl);
    if (wrapper.IsEmpty()) {
        wrapper = V8Screen::wrap(impl);
        if (!wrapper.IsEmpty())
            V8DOMWrapper::setHiddenWindowReference(impl->frame(), V8DOMWindow::screenIndex, wrapper);
    }
    return wrapper;
}

} // namespace WebCore
