

#include "config.h"
#include "V8BarInfo.h"

#include "V8DOMWindow.h"
#include "V8DOMWrapper.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(BarInfo* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = getDOMObjectMap().get(impl);
    if (wrapper.IsEmpty()) {
        wrapper = V8BarInfo::wrap(impl);
        if (!wrapper.IsEmpty()) {
            Frame* frame = impl->frame();
            switch (impl->type()) {
            case BarInfo::Locationbar:
                V8DOMWrapper::setHiddenWindowReference(frame, V8DOMWindow::locationbarIndex, wrapper);
                break;
            case BarInfo::Menubar:
                V8DOMWrapper::setHiddenWindowReference(frame, V8DOMWindow::menubarIndex, wrapper);
                break;
            case BarInfo::Personalbar:
                V8DOMWrapper::setHiddenWindowReference(frame, V8DOMWindow::personalbarIndex, wrapper);
                break;
            case BarInfo::Scrollbars:
                V8DOMWrapper::setHiddenWindowReference(frame, V8DOMWindow::scrollbarsIndex, wrapper);
                break;
            case BarInfo::Statusbar:
                V8DOMWrapper::setHiddenWindowReference(frame, V8DOMWindow::statusbarIndex, wrapper);
                break;
            case BarInfo::Toolbar:
                V8DOMWrapper::setHiddenWindowReference(frame, V8DOMWindow::toolbarIndex, wrapper);
                break;
            }
        }
    }
    return wrapper;
}

} // namespace WebCore
