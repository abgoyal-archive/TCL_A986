

#include "config.h"
#include "V8History.h"

#include "ExceptionCode.h"
#include "History.h"
#include "SerializedScriptValue.h"
#include "V8Binding.h"
#include "V8BindingState.h"
#include "V8DOMWindow.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8History::pushStateCallback(const v8::Arguments& args)
{
    RefPtr<SerializedScriptValue> historyState = SerializedScriptValue::create(args[0]);

    v8::TryCatch tryCatch;
    String title = toWebCoreStringWithNullOrUndefinedCheck(args[1]);
    if (tryCatch.HasCaught())
        return v8::Undefined();
    String url;
    if (args.Length() > 2) {
        url = toWebCoreStringWithNullOrUndefinedCheck(args[2]);
        if (tryCatch.HasCaught())
            return v8::Undefined();
    }

    ExceptionCode ec = 0;
    History* history = V8History::toNative(args.Holder());
    history->stateObjectAdded(historyState.release(), title, url, History::StateObjectPush, ec);
    return throwError(ec);
}

v8::Handle<v8::Value> V8History::replaceStateCallback(const v8::Arguments& args)
{
    RefPtr<SerializedScriptValue> historyState = SerializedScriptValue::create(args[0]);

    v8::TryCatch tryCatch;
    String title = toWebCoreStringWithNullOrUndefinedCheck(args[1]);
    if (tryCatch.HasCaught())
        return v8::Undefined();
    String url;
    if (args.Length() > 2) {
        url = toWebCoreStringWithNullOrUndefinedCheck(args[2]);
        if (tryCatch.HasCaught())
            return v8::Undefined();
    }

    ExceptionCode ec = 0;
    History* history = V8History::toNative(args.Holder());
    history->stateObjectAdded(historyState.release(), title, url, History::StateObjectReplace, ec);
    return throwError(ec);
}

bool V8History::indexedSecurityCheck(v8::Local<v8::Object> host, uint32_t index, v8::AccessType type, v8::Local<v8::Value> data)
{
    ASSERT(V8ClassIndex::FromInt(data->Int32Value()) == V8ClassIndex::HISTORY);
    // Only allow same origin access.
    History* history = V8History::toNative(host);
    return V8BindingSecurity::canAccessFrame(V8BindingState::Only(), history->frame(), false);
}

bool V8History::namedSecurityCheck(v8::Local<v8::Object> host, v8::Local<v8::Value> key, v8::AccessType type, v8::Local<v8::Value> data)
{
    ASSERT(V8ClassIndex::FromInt(data->Int32Value()) == V8ClassIndex::HISTORY);
    // Only allow same origin access.
    History* history = V8History::toNative(host);
    return V8BindingSecurity::canAccessFrame(V8BindingState::Only(), history->frame(), false);
}

v8::Handle<v8::Value> toV8(History* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = getDOMObjectMap().get(impl);
    if (wrapper.IsEmpty()) {
        wrapper = V8History::wrap(impl);
        if (!wrapper.IsEmpty())
            V8DOMWrapper::setHiddenWindowReference(impl->frame(), V8DOMWindow::historyIndex, wrapper);
    }
    return wrapper;
}

} // namespace WebCore
