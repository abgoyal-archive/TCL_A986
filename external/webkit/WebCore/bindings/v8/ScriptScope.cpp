

#include "config.h"
#include "ScriptScope.h"

#include "ScriptState.h"

#include <v8.h>

namespace WebCore {

ScriptScope::ScriptScope(ScriptState* scriptState, bool reportExceptions)
    : m_context(scriptState->context())
    , m_scope(m_context)
    , m_scriptState(scriptState)
{
    m_exceptionCatcher.SetVerbose(reportExceptions);
    ASSERT(!m_context.IsEmpty());
}

bool ScriptScope::success()
{
    if (!m_exceptionCatcher.HasCaught())
        return true;
    m_exceptionCatcher.Reset();
    return false;
}

} // namespace WebCore
