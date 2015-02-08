

#ifndef JavaScriptCallFrame_h
#define JavaScriptCallFrame_h

#if ENABLE(JAVASCRIPT_DEBUGGER) && USE(JSC)

#include <interpreter/CallFrame.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <debugger/DebuggerCallFrame.h>

namespace WebCore {

    class String;

    class JavaScriptCallFrame : public RefCounted<JavaScriptCallFrame> {
    public:
        static PassRefPtr<JavaScriptCallFrame> create(const JSC::DebuggerCallFrame& debuggerCallFrame, PassRefPtr<JavaScriptCallFrame> caller, intptr_t sourceID, int line)
        {
            return adoptRef(new JavaScriptCallFrame(debuggerCallFrame, caller, sourceID, line));
        }

        void invalidate()
        {
            m_isValid = false;
            m_debuggerCallFrame = 0;
        }

        bool isValid() const { return m_isValid; }

        JavaScriptCallFrame* caller();

        intptr_t sourceID() const { return m_sourceID; }
        int line() const { return m_line; }
        void update(const JSC::DebuggerCallFrame& debuggerCallFrame, intptr_t sourceID, int line)
        {
            m_debuggerCallFrame = debuggerCallFrame;
            m_line = line;
            m_sourceID = sourceID;
            m_isValid = true;
        }

        String functionName() const;
        JSC::DebuggerCallFrame::Type type() const;
        const JSC::ScopeChainNode* scopeChain() const;
        JSC::JSGlobalObject* dynamicGlobalObject() const;

        JSC::JSObject* thisObject() const;
        JSC::JSValue evaluate(const JSC::UString& script, JSC::JSValue& exception) const;
        
    private:
        JavaScriptCallFrame(const JSC::DebuggerCallFrame&, PassRefPtr<JavaScriptCallFrame> caller, intptr_t sourceID, int line);

        JSC::DebuggerCallFrame m_debuggerCallFrame;
        RefPtr<JavaScriptCallFrame> m_caller;
        intptr_t m_sourceID;
        int m_line;
        bool m_isValid;
    };

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)

#endif // JavaScriptCallFrame_h
