

#include "config.h"
#include "JSGeolocation.h"

#include "DOMWindow.h"
#include "ExceptionCode.h"
#include "Geolocation.h"
#include "GeolocationService.h"
#include "JSCustomPositionCallback.h"
#include "JSCustomPositionErrorCallback.h"
#include "JSDOMWindow.h"
#include "PositionOptions.h"
#include <runtime/InternalFunction.h>

using namespace JSC;
using namespace std;

namespace WebCore {

static PassRefPtr<PositionCallback> createPositionCallback(ExecState* exec, JSDOMGlobalObject* globalObject, JSValue value)
{
    // The spec specifies 'FunctionOnly' for this object.
    if (!value.inherits(&InternalFunction::info)) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return 0;
    }

    JSObject* object = asObject(value);
    return JSCustomPositionCallback::create(object, globalObject);
}

static PassRefPtr<PositionErrorCallback> createPositionErrorCallback(ExecState* exec, JSDOMGlobalObject* globalObject, JSValue value)
{
    // Argument is optional (hence undefined is allowed), and null is allowed.
    if (value.isUndefinedOrNull())
        return 0;

    // The spec specifies 'FunctionOnly' for this object.
    if (!value.inherits(&InternalFunction::info)) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return 0;
    }

    JSObject* object = asObject(value);
    return JSCustomPositionErrorCallback::create(object, globalObject);
}

static PassRefPtr<PositionOptions> createPositionOptions(ExecState* exec, JSValue value)
{
    // Create default options.
    RefPtr<PositionOptions> options = PositionOptions::create();

    // Argument is optional (hence undefined is allowed), and null is allowed.
    if (value.isUndefinedOrNull()) {
        // Use default options.
        return options.release();
    }

    // Given the above test, this will always yield an object.
    JSObject* object = value.toObject(exec);

    // For all three properties, we apply the following ...
    // - If the getter or the property's valueOf method throws an exception, we
    //   quit so as not to risk overwriting the exception.
    // - If the value is absent or undefined, we don't override the default.
    JSValue enableHighAccuracyValue = object->get(exec, Identifier(exec, "enableHighAccuracy"));
    if (exec->hadException())
        return 0;
    if (!enableHighAccuracyValue.isUndefined()) {
        options->setEnableHighAccuracy(enableHighAccuracyValue.toBoolean(exec));
        if (exec->hadException())
            return 0;
    }

    JSValue timeoutValue = object->get(exec, Identifier(exec, "timeout"));
    if (exec->hadException())
        return 0;
    if (!timeoutValue.isUndefined()) {
        double timeoutNumber = timeoutValue.toNumber(exec);
        if (exec->hadException())
            return 0;
        // If the value is positive infinity, there's nothing to do.
        if (!(isinf(timeoutNumber) && (timeoutNumber > 0))) {
            // Wrap to int32 and force non-negative to match behavior of window.setTimeout.
            options->setTimeout(max(0, timeoutValue.toInt32(exec)));
            if (exec->hadException())
                return 0;
        }
    }

    JSValue maximumAgeValue = object->get(exec, Identifier(exec, "maximumAge"));
    if (exec->hadException())
        return 0;
    if (!maximumAgeValue.isUndefined()) {
        double maximumAgeNumber = maximumAgeValue.toNumber(exec);
        if (exec->hadException())
            return 0;
        if (isinf(maximumAgeNumber) && (maximumAgeNumber > 0)) {
            // If the value is positive infinity, clear maximumAge.
            options->clearMaximumAge();
        } else {
            // Wrap to int32 and force non-negative to match behavior of window.setTimeout.
            options->setMaximumAge(max(0, maximumAgeValue.toInt32(exec)));
            if (exec->hadException())
                return 0;
        }
    }

    return options.release();
}

JSValue JSGeolocation::getCurrentPosition(ExecState* exec, const ArgList& args)
{
    // Arguments: PositionCallback, (optional)PositionErrorCallback, (optional)PositionOptions

    RefPtr<PositionCallback> positionCallback = createPositionCallback(exec, static_cast<JSDOMGlobalObject*>(exec->lexicalGlobalObject()), args.at(0));
    if (exec->hadException())
        return jsUndefined();
    ASSERT(positionCallback);

    RefPtr<PositionErrorCallback> positionErrorCallback = createPositionErrorCallback(exec, static_cast<JSDOMGlobalObject*>(exec->lexicalGlobalObject()), args.at(1));
    if (exec->hadException())
        return jsUndefined();

    RefPtr<PositionOptions> positionOptions = createPositionOptions(exec, args.at(2));
    if (exec->hadException())
        return jsUndefined();
    ASSERT(positionOptions);

    m_impl->getCurrentPosition(positionCallback.release(), positionErrorCallback.release(), positionOptions.release());
    return jsUndefined();
}

JSValue JSGeolocation::watchPosition(ExecState* exec, const ArgList& args)
{
    // Arguments: PositionCallback, (optional)PositionErrorCallback, (optional)PositionOptions

    RefPtr<PositionCallback> positionCallback = createPositionCallback(exec, static_cast<JSDOMGlobalObject*>(exec->lexicalGlobalObject()), args.at(0));
    if (exec->hadException())
        return jsUndefined();
    ASSERT(positionCallback);

    RefPtr<PositionErrorCallback> positionErrorCallback = createPositionErrorCallback(exec, static_cast<JSDOMGlobalObject*>(exec->lexicalGlobalObject()), args.at(1));
    if (exec->hadException())
        return jsUndefined();

    RefPtr<PositionOptions> positionOptions = createPositionOptions(exec, args.at(2));
    if (exec->hadException())
        return jsUndefined();
    ASSERT(positionOptions);

    int watchID = m_impl->watchPosition(positionCallback.release(), positionErrorCallback.release(), positionOptions.release());
    return jsNumber(exec, watchID);
}

} // namespace WebCore
