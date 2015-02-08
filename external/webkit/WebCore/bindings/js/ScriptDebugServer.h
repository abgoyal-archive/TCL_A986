

#ifndef ScriptDebugServer_h
#define ScriptDebugServer_h

#if ENABLE(JAVASCRIPT_DEBUGGER)

#include <wtf/Noncopyable.h>

namespace WebCore {

class ScriptDebugServer : public Noncopyable {
public:
    static void recompileAllJSFunctions();
    static void recompileAllJSFunctionsSoon();
};

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)

#endif // ScriptDebugServer_h
