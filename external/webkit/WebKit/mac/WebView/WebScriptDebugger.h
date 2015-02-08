

#ifndef WebScriptDebugger_h
#define WebScriptDebugger_h

#include <debugger/Debugger.h>
#include <runtime/Protect.h>

#include <wtf/RetainPtr.h>

namespace JSC {
    class DebuggerCallFrame;
    class ExecState;
    class JSGlobalObject;
    class JSObject;
    class ArgList;
    class UString;
}

@class WebScriptCallFrame;

NSString *toNSString(const JSC::UString&);

class WebScriptDebugger : public JSC::Debugger {
public:
    WebScriptDebugger(JSC::JSGlobalObject*);

    void initGlobalCallFrame(const JSC::DebuggerCallFrame&);

    virtual void sourceParsed(JSC::ExecState*, const JSC::SourceCode&, int errorLine, const JSC::UString& errorMsg);
    virtual void callEvent(const JSC::DebuggerCallFrame&, intptr_t sourceID, int lineNumber);
    virtual void atStatement(const JSC::DebuggerCallFrame&, intptr_t sourceID, int lineNumber);
    virtual void returnEvent(const JSC::DebuggerCallFrame&, intptr_t sourceID, int lineNumber);
    virtual void exception(const JSC::DebuggerCallFrame&, intptr_t sourceID, int lineNumber, bool hasHandler);
    virtual void willExecuteProgram(const JSC::DebuggerCallFrame&, intptr_t sourceID, int lineno);
    virtual void didExecuteProgram(const JSC::DebuggerCallFrame&, intptr_t sourceID, int lineno);
    virtual void didReachBreakpoint(const JSC::DebuggerCallFrame&, intptr_t sourceID, int lineno);

    JSC::JSGlobalObject* globalObject() const { return m_globalObject.get(); }
    WebScriptCallFrame *globalCallFrame() const { return m_globalCallFrame.get(); }

private:
    bool m_callingDelegate;
    RetainPtr<WebScriptCallFrame> m_topCallFrame;

    JSC::ProtectedPtr<JSC::JSGlobalObject> m_globalObject;
    RetainPtr<WebScriptCallFrame> m_globalCallFrame;
};

#endif // WebScriptDebugger_h
