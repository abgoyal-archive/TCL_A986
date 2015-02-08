

#include "config.h"

#if ENABLE(WORKERS)

#include "JSAbstractWorker.h"

#include "AbstractWorker.h"
#include "JSDOMGlobalObject.h"
#include "JSEventListener.h"
#include "JSEventTarget.h"

using namespace JSC;

namespace WebCore {

JSValue JSAbstractWorker::addEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->addEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSAbstractWorker::removeEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->removeEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
