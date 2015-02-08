

#include "config.h"

#if ENABLE(WEB_SOCKETS)

#include "JSWebSocketConstructor.h"

#include "JSWebSocket.h"
#include "ScriptExecutionContext.h"
#include "WebSocket.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

ASSERT_CLASS_FITS_IN_CELL(JSWebSocketConstructor);

const ClassInfo JSWebSocketConstructor::s_info = { "WebSocketConstructor", 0, 0, 0 };

JSWebSocketConstructor::JSWebSocketConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebSocketConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebSocketPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 1), ReadOnly | DontDelete | DontEnum);
}

static JSObject* constructWebSocket(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebSocketConstructor* jsConstructor = static_cast<JSWebSocketConstructor*>(constructor);
    ScriptExecutionContext* context = jsConstructor->scriptExecutionContext();
    if (!context)
        return throwError(exec, ReferenceError, "WebSocket constructor associated document is unavailable");

    if (args.size() == 0)
        return throwError(exec, SyntaxError, "Not enough arguments");

    const String& urlString = args.at(0).toString(exec);
    if (exec->hadException())
        return throwError(exec, SyntaxError, "wrong URL");
    const KURL& url = context->completeURL(urlString);
    RefPtr<WebSocket> webSocket = WebSocket::create(context);
    ExceptionCode ec = 0;
    if (args.size() < 2)
        webSocket->connect(url, ec);
    else {
        const String& protocol = args.at(1).toString(exec);
        if (exec->hadException())
            return 0;
        webSocket->connect(url, protocol, ec);
    }
    setDOMException(exec, ec);
    return CREATE_DOM_OBJECT_WRAPPER(exec, jsConstructor->globalObject(), WebSocket, webSocket.get());
}

ConstructType JSWebSocketConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructWebSocket;
    return ConstructTypeHost;
}

}  // namespace WebCore

#endif
