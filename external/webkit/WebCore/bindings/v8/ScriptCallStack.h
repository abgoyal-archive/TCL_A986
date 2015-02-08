

#ifndef ScriptCallStack_h
#define ScriptCallStack_h

#include "ScriptCallFrame.h"
#include "ScriptState.h"
#include "ScriptValue.h"
#include <wtf/Noncopyable.h>

namespace v8 {
    class Arguments;
}

namespace WebCore {

    class ScriptCallStack : public Noncopyable {
    public:
        static ScriptCallStack* create(const v8::Arguments&, unsigned skipArgumentCount = 0);
        ~ScriptCallStack();

        const ScriptCallFrame& at(unsigned) const;
        // FIXME: implement retrieving and storing call stack trace
        unsigned size() const { return 1; }

        ScriptState* state() const { return m_scriptState; }
        ScriptState* globalState() const { return m_scriptState; }

    private:
        ScriptCallStack(const v8::Arguments& arguments, unsigned skipArgumentCount, String sourceName, int sourceLineNumber);
    
        ScriptCallFrame m_lastCaller;
        ScriptState* m_scriptState;
    };

} // namespace WebCore

#endif // ScriptCallStack_h
