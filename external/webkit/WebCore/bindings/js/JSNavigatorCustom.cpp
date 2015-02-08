

#include "config.h"
#include "JSNavigator.h"

#include "ExceptionCode.h"
#include "Navigator.h"
#include <runtime/InternalFunction.h>

#if PLATFORM(ANDROID)
#include "JSCustomApplicationInstalledCallback.h"
#endif

namespace WebCore {

using namespace JSC;

void JSNavigator::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    JSGlobalData& globalData = *Heap::heap(this)->globalData();

    markDOMObjectWrapper(markStack, globalData, impl()->optionalGeolocation());
}

#if PLATFORM(ANDROID) && ENABLE(APPLICATION_INSTALLED)

JSC::JSValue  WebCore::JSNavigator::isApplicationInstalled(JSC::ExecState* exec, JSC::ArgList const& args)
{
    if (args.size() < 2) {
        setDOMException(exec, SYNTAX_ERR);
        return jsUndefined();
    }

    if (!args.at(1).inherits(&InternalFunction::info)) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }

    String appName = args.at(0).toString(exec);

    JSObject* object;
    if (!(object = args.at(1).getObject())) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }

    RefPtr<ApplicationInstalledCallback> callback(JSCustomApplicationInstalledCallback::create(
            object, static_cast<JSDOMGlobalObject*>(exec->dynamicGlobalObject())));

    if (!m_impl->isApplicationInstalled(appName, callback.release()))
        setDOMException(exec, INVALID_STATE_ERR);
    return jsUndefined();
}

#endif

}
