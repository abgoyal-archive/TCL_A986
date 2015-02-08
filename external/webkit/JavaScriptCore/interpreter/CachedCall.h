

#ifndef CachedCall_h
#define CachedCall_h

#include "CallFrameClosure.h"
#include "JSFunction.h"
#include "JSGlobalObject.h"
#include "Interpreter.h"

namespace JSC {
    class CachedCall : public Noncopyable {
    public:
        CachedCall(CallFrame* callFrame, JSFunction* function, int argCount, JSValue* exception)
            : m_valid(false)
            , m_interpreter(callFrame->interpreter())
            , m_exception(exception)
            , m_globalObjectScope(callFrame, function->scope().globalObject())
        {
            ASSERT(!function->isHostFunction());
            m_closure = m_interpreter->prepareForRepeatCall(function->jsExecutable(), callFrame, function, argCount, function->scope().node(), exception);
            m_valid = !*exception;
        }
        
        JSValue call()
        { 
            ASSERT(m_valid);
            return m_interpreter->execute(m_closure, m_exception);
        }
        void setThis(JSValue v) { m_closure.setArgument(0, v); }
        void setArgument(int n, JSValue v) { m_closure.setArgument(n + 1, v); }

        CallFrame* newCallFrame(ExecState* exec)
        {
            CallFrame* callFrame = m_closure.newCallFrame;
            callFrame->setScopeChain(exec->scopeChain());
            return callFrame;
        }

        ~CachedCall()
        {
            if (m_valid)
                m_interpreter->endRepeatCall(m_closure);
        }
        
    private:
        bool m_valid;
        Interpreter* m_interpreter;
        JSValue* m_exception;
        DynamicGlobalObjectScope m_globalObjectScope;
        CallFrameClosure m_closure;
    };
}

#endif
