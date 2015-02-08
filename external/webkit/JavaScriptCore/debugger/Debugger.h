

#ifndef Debugger_h
#define Debugger_h

#include <wtf/HashSet.h>

namespace JSC {

    class DebuggerCallFrame;
    class ExecState;
    class JSGlobalData;
    class JSGlobalObject;
    class JSValue;
    class SourceCode;
    class UString;

    class Debugger {
    public:
        virtual ~Debugger();

        void attach(JSGlobalObject*);
        virtual void detach(JSGlobalObject*);

        virtual void sourceParsed(ExecState*, const SourceCode&, int errorLineNumber, const UString& errorMessage) = 0;
        virtual void exception(const DebuggerCallFrame&, intptr_t sourceID, int lineNumber, bool hasHandler) = 0;
        virtual void atStatement(const DebuggerCallFrame&, intptr_t sourceID, int lineNumber) = 0;
        virtual void callEvent(const DebuggerCallFrame&, intptr_t sourceID, int lineNumber) = 0;
        virtual void returnEvent(const DebuggerCallFrame&, intptr_t sourceID, int lineNumber) = 0;

        virtual void willExecuteProgram(const DebuggerCallFrame&, intptr_t sourceID, int lineNumber) = 0;
        virtual void didExecuteProgram(const DebuggerCallFrame&, intptr_t sourceID, int lineNumber) = 0;
        virtual void didReachBreakpoint(const DebuggerCallFrame&, intptr_t sourceID, int lineNumber) = 0;

        void recompileAllJSFunctions(JSGlobalData*);

    private:
        HashSet<JSGlobalObject*> m_globalObjects;
    };

    // This function exists only for backwards compatibility with existing WebScriptDebugger clients.
    JSValue evaluateInGlobalCallFrame(const UString&, JSValue& exception, JSGlobalObject*);

} // namespace JSC

#endif // Debugger_h
