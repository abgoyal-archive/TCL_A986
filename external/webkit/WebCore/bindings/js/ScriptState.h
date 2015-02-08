

#ifndef ScriptState_h
#define ScriptState_h

#include "JSDOMBinding.h"

namespace WebCore {
    class DOMWrapperWorld;
    class Frame;
    class Node;
    class Page;

    // The idea is to expose "state-like" methods (hadException, and any other
    // methods where ExecState just dips into globalData) of JSC::ExecState as a
    // separate abstraction.
    // For now, the separation is purely by convention.
    typedef JSC::ExecState ScriptState;

    ScriptState* mainWorldScriptState(Frame*);

    ScriptState* scriptStateFromNode(DOMWrapperWorld*, Node*);
    ScriptState* scriptStateFromPage(DOMWrapperWorld*, Page*);

} // namespace WebCore

#endif // ScriptState_h
