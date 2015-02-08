

#ifndef ScriptCallFrame_h
#define ScriptCallFrame_h

#include "KURL.h"
#include <runtime/ArgList.h>
#include "ScriptString.h"
#include "ScriptValue.h"
#include <wtf/Vector.h>

namespace JSC {
    class ExecState;
    class InternalFunction;
}

namespace WebCore {

    // FIXME: Implement retrieving line number and source URL and storing here
    // for all call frames, not just the first one.
    // See <https://bugs.webkit.org/show_bug.cgi?id=22556> and
    // <https://bugs.webkit.org/show_bug.cgi?id=21180>
    class ScriptCallFrame  {
    public:
        ScriptCallFrame(const JSC::UString& functionName, const JSC::UString& urlString, int lineNumber, const JSC::ArgList&, unsigned skipArgumentCount);
        ~ScriptCallFrame();

        const ScriptString& functionName() const { return m_functionName; }
        const KURL& sourceURL() const { return m_sourceURL; }
        unsigned lineNumber() const { return m_lineNumber; }

        // argument retrieval methods
        const ScriptValue& argumentAt(unsigned) const;
        unsigned argumentCount() const { return m_arguments.size(); }

    private:
        ScriptString m_functionName;
        KURL m_sourceURL;
        unsigned m_lineNumber;

        Vector<ScriptValue> m_arguments;
    };

} // namespace WebCore

#endif // ScriptCallFrame_h
