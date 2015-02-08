

#include "config.h"
#include "XPathVariableReference.h"

#if ENABLE(XPATH)

#include "Node.h"
#include "XPathValue.h"

namespace WebCore {
namespace XPath {
    
VariableReference::VariableReference(const String& name)
    : m_name(name)
{
}

Value VariableReference::evaluate() const
{
    HashMap<String, String>& bindings = evaluationContext().variableBindings;
    if (!bindings.contains(m_name))
        // FIXME: Is this the right thing to do if an unknown variable is referenced?
        return "";
    return bindings.get(m_name);
}

}
}

#endif // ENABLE(XPATH)
