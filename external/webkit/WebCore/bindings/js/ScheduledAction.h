

#ifndef ScheduledAction_h
#define ScheduledAction_h

#include "PlatformString.h"
#include <JSDOMBinding.h>
#include <runtime/JSCell.h>
#include <runtime/Protect.h>
#include <wtf/Vector.h>

namespace JSC {
    class JSGlobalObject;
}

namespace WebCore {

    class Document;
    class ScriptExecutionContext;
    class WorkerContext;

   /* An action (either function or string) to be executed after a specified
    * time interval, either once or repeatedly. Used for window.setTimeout()
    * and window.setInterval()
    */
    class ScheduledAction : public Noncopyable {
    public:
        static ScheduledAction* create(JSC::ExecState*, const JSC::ArgList&, DOMWrapperWorld* isolatedWorld);

        void execute(ScriptExecutionContext*);

    private:
        ScheduledAction(JSC::JSValue function, const JSC::ArgList&, DOMWrapperWorld* isolatedWorld);
        ScheduledAction(const String& code, DOMWrapperWorld* isolatedWorld)
            : m_code(code)
            , m_isolatedWorld(isolatedWorld)
        {
        }

        void executeFunctionInContext(JSC::JSGlobalObject*, JSC::JSValue thisValue);
        void execute(Document*);
#if ENABLE(WORKERS)        
        void execute(WorkerContext*);
#endif

        JSC::ProtectedJSValue m_function;
        Vector<JSC::ProtectedJSValue> m_args;
        String m_code;
        RefPtr<DOMWrapperWorld> m_isolatedWorld;
    };

} // namespace WebCore

#endif // ScheduledAction_h
