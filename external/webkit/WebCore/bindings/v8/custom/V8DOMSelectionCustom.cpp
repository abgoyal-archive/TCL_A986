

#include "config.h"
#include "V8DOMSelection.h"

#include "V8DOMWindow.h"
#include "V8DOMWrapper.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(DOMSelection* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = getDOMObjectMap().get(impl);
    if (wrapper.IsEmpty()) {
        wrapper = V8DOMSelection::wrap(impl);
        V8DOMWrapper::setHiddenWindowReference(impl->frame(), V8DOMWindow::domSelectionIndex, wrapper);
    }
    return wrapper;
}

} // namespace WebCore
