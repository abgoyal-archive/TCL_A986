
 
#ifndef DebuggerCallFrame_h
#define DebuggerCallFrame_h

#include "CallFrame.h"

namespace JSC {
    
    class DebuggerCallFrame {
    public:
        enum Type { ProgramType, FunctionType };

        DebuggerCallFrame(CallFrame* callFrame)
            : m_callFrame(callFrame)
        {
        }

        DebuggerCallFrame(CallFrame* callFrame, JSValue exception)
            : m_callFrame(callFrame)
            , m_exception(exception)
        {
        }

        JSGlobalObject* dynamicGlobalObject() const { return m_callFrame->dynamicGlobalObject(); }
        const ScopeChainNode* scopeChain() const { return m_callFrame->scopeChain(); }
        const UString* functionName() const;
        UString calculatedFunctionName() const;
        Type type() const;
        JSObject* thisObject() const;
        JSValue evaluate(const UString&, JSValue& exception) const;
        JSValue exception() const { return m_exception; }

    private:
        CallFrame* m_callFrame;
        JSValue m_exception;
    };

} // namespace JSC

#endif // DebuggerCallFrame_h
