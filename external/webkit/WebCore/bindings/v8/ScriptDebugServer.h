

#ifndef ScriptDebugServer_h
#define ScriptDebugServer_h

#include <wtf/Noncopyable.h>

namespace WebCore {

class ScriptDebugServer : public Noncopyable {
public:
    static void recompileAllJSFunctions();
    static void recompileAllJSFunctionsSoon();
};

} // namespace WebCore

#endif // ScriptDebugServer_h
