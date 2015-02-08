

#ifndef InjectedScript_h
#define InjectedScript_h

#include "InjectedScriptHost.h"
#include "ScriptObject.h"
#include <wtf/Noncopyable.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

class SerializedScriptValue;
class String;

class InjectedScript {
public:
    InjectedScript() { }
    ~InjectedScript() { }

    bool hasNoValue() const { return m_injectedScriptObject.hasNoValue(); }

    void dispatch(long callId, const String& methodName, const String& arguments, bool async, RefPtr<SerializedScriptValue>* result, bool* hadException);
#if ENABLE(JAVASCRIPT_DEBUGGER)
    PassRefPtr<SerializedScriptValue> callFrames();
#endif
    PassRefPtr<SerializedScriptValue> wrapForConsole(ScriptValue);
    void releaseWrapperObjectGroup(const String&);

private:
    friend InjectedScript InjectedScriptHost::injectedScriptFor(ScriptState*);
    explicit InjectedScript(ScriptObject);
    ScriptObject m_injectedScriptObject;
};

} // namespace WebCore

#endif
