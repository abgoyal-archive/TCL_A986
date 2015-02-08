

#include "config.h"
#include "ScriptCallFrame.h"

#include <interpreter/CallFrame.h>
#include <runtime/UString.h>

using namespace JSC;

namespace WebCore {

ScriptCallFrame::ScriptCallFrame(const UString& functionName, const UString& urlString, int lineNumber, const ArgList& args, unsigned skipArgumentCount)
    : m_functionName(functionName)
    , m_sourceURL(ParsedURLString, urlString)
    , m_lineNumber(lineNumber)
{
    size_t argumentCount = args.size();
    for (size_t i = skipArgumentCount; i < argumentCount; ++i)
        m_arguments.append(ScriptValue(args.at(i)));
}

ScriptCallFrame::~ScriptCallFrame()
{
}

const ScriptValue &ScriptCallFrame::argumentAt(unsigned index) const
{
    ASSERT(m_arguments.size() > index);
    return m_arguments[index];
}

} // namespace WebCore
