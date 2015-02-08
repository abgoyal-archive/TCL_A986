

#include "config.h"

#if ENABLE(WEB_SOCKETS)

#include "JSWebSocket.h"

#include "KURL.h"
#include "JSEventListener.h"
#include "WebSocket.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

// Custom functions
JSValue JSWebSocket::send(ExecState* exec, const ArgList& args)
{
    if (args.size() < 1)
        return throwError(exec, SyntaxError, "Not enough arguments");

    const String& msg = args.at(0).toString(exec);
    if (exec->hadException())
        return throwError(exec, SyntaxError, "bad message data.");
    ExceptionCode ec = 0;
    JSValue ret = jsBoolean(impl()->send(msg, ec));
    setDOMException(exec, ec);
    return ret;
}

JSValue JSWebSocket::addEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->addEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSWebSocket::removeEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->removeEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

}  // namespace WebCore

#endif
