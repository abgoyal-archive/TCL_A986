

#ifndef ScriptCallStack_h
#define ScriptCallStack_h

#include "ScriptCallFrame.h"
#include "ScriptState.h"
#include "ScriptString.h"
#include <wtf/Noncopyable.h>

namespace JSC {
    class ExecState;
    class JSValue;
}

namespace WebCore {

    class ScriptCallStack : public Noncopyable {
    public:
        ScriptCallStack(JSC::ExecState*, const JSC::ArgList&, unsigned skipArgumentCount = 0);
        ~ScriptCallStack();

        ScriptState* state() const { return m_exec; }
        ScriptState* globalState() const { return m_exec->lexicalGlobalObject()->globalExec(); }
        // frame retrieval methods
        const ScriptCallFrame &at(unsigned);
        unsigned size();

    private:
        void initialize();
        bool m_initialized;

        JSC::ExecState* m_exec;
        Vector<ScriptCallFrame> m_frames;
        JSC::InternalFunction* m_caller;
    };

} // namespace WebCore

#endif // ScriptCallStack_h
