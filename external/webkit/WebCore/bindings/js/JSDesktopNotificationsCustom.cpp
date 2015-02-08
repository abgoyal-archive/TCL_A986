

#include "config.h"

#if ENABLE(NOTIFICATIONS)

#include "Document.h"
#include "JSCustomVoidCallback.h"
#include "JSEventListener.h"
#include "JSNotification.h"
#include "JSNotificationCenter.h"
#include "Notification.h"
#include "NotificationCenter.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

JSValue JSNotificationCenter::requestPermission(ExecState* exec, const ArgList& args)
{
    // Permission request is only valid from page context.
    ScriptExecutionContext* context = impl()->context();
    if (context->isWorkerContext())
        return throwError(exec, SyntaxError);

    if (!args.at(0).isObject())
        return throwError(exec, TypeError);

    PassRefPtr<JSCustomVoidCallback> callback = JSCustomVoidCallback::create(args.at(0).getObject(), static_cast<Document*>(context)->frame());

    impl()->requestPermission(callback);
    return jsUndefined();
}

JSValue JSNotification::addEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->addEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener)), false, currentWorld(exec)), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSNotification::removeEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->removeEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}


} // namespace

#endif // ENABLE(NOTIFICATIONS)
