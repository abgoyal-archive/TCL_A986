

#include "config.h"
#include "V8Navigator.h"

#include "RuntimeEnabledFeatures.h"
#include "V8DOMWindow.h"
#include "V8DOMWrapper.h"

#if PLATFORM(ANDROID)
#include "ExceptionCode.h"
#include "V8CustomApplicationInstalledCallback.h"
#include "V8Proxy.h"
#endif

namespace WebCore {

v8::Handle<v8::Value> toV8(Navigator* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = getDOMObjectMap().get(impl);
    if (wrapper.IsEmpty()) {
        wrapper = V8Navigator::wrap(impl);
        if (!wrapper.IsEmpty())
            V8DOMWrapper::setHiddenWindowReference(impl->frame(), V8DOMWindow::navigatorIndex, wrapper);
    }
    return wrapper;
}

#if PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)

static PassRefPtr<ApplicationInstalledCallback> createApplicationInstalledCallback(
        v8::Local<v8::Value> value, bool& succeeded)
{
    succeeded = true;

    if (!value->IsFunction()) {
        succeeded = false;
        throwError("The second argument should be a function");
        return 0;
    }

    Frame* frame = V8Proxy::retrieveFrameForCurrentContext();
    return V8CustomApplicationInstalledCallback::create(value, frame);
}

v8::Handle<v8::Value> V8Navigator::isApplicationInstalledCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.isApplicationInstalled()");
    bool succeeded = false;

    if (args.Length() < 2)
        return throwError("Two arguments required: an application name and a callback.", V8Proxy::SyntaxError);

    if (!args[0]->IsString())
        return throwError("The first argument should be a string.");

    RefPtr<ApplicationInstalledCallback> callback =
        createApplicationInstalledCallback(args[1], succeeded);
    if (!succeeded)
        return v8::Undefined();

    ASSERT(callback);

    Navigator* navigator = V8Navigator::toNative(args.Holder());
    if (!navigator->isApplicationInstalled(toWebCoreString(args[0]), callback.release()))
        return throwError(INVALID_STATE_ERR);

    return v8::Undefined();
}

#endif // PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)

} // namespace WebCore
