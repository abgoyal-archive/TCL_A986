

#include "config.h"

#if ENABLE(JAVASCRIPT_DEBUGGER)

#include "ScriptDebugServer.h"

#include "JavaScriptDebugServer.h"

namespace WebCore {

void ScriptDebugServer::recompileAllJSFunctions()
{
    JavaScriptDebugServer::shared().recompileAllJSFunctions();
}

void ScriptDebugServer::recompileAllJSFunctionsSoon()
{
    JavaScriptDebugServer::shared().recompileAllJSFunctionsSoon();
}

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)
