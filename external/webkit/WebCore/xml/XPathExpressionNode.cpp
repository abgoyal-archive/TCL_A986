

#include "config.h"
#include "XPathExpressionNode.h"

#if ENABLE(XPATH)

#include "Node.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {
namespace XPath {
    
EvaluationContext& Expression::evaluationContext()
{
    DEFINE_STATIC_LOCAL(EvaluationContext, evaluationContext, ());
    return evaluationContext;
}

Expression::Expression()
    : m_isContextNodeSensitive(false)
    , m_isContextPositionSensitive(false)
    , m_isContextSizeSensitive(false)
{
}

Expression::~Expression()
{
    deleteAllValues(m_subExpressions);
}

}
}

#endif // ENABLE(XPATH)
