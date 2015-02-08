

#include "config.h"
#include "ScriptCallFrame.h"

#include <v8.h>

#include "PlatformString.h"
#include "V8Binding.h"
#include "V8Proxy.h"
#include "ScriptValue.h"

namespace WebCore {

ScriptCallFrame::ScriptCallFrame(const String& functionName, const String& urlString, int lineNumber, const v8::Arguments& arguments, unsigned skipArgumentCount)
    : m_functionName(functionName)
    , m_sourceURL(ParsedURLString, urlString)
    , m_lineNumber(lineNumber)
{
    for (int i = 0; i < arguments.Length(); ++i)
        m_arguments.append(ScriptValue(arguments[i]));
}

ScriptCallFrame::~ScriptCallFrame()
{
}

const ScriptValue& ScriptCallFrame::argumentAt(unsigned index) const
{
    ASSERT(m_arguments.size() > index);
    return m_arguments[index];
}

} // namespace WebCore
