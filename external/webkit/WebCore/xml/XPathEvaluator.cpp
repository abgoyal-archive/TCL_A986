

#include "config.h"
#include "XPathEvaluator.h"

#if ENABLE(XPATH)

#include "ExceptionCode.h"
#include "NativeXPathNSResolver.h"
#include "Node.h"
#include "XPathExpression.h"
#include "XPathResult.h"
#include "XPathUtil.h"

namespace WebCore {

using namespace XPath;
    
PassRefPtr<XPathExpression> XPathEvaluator::createExpression(const String& expression,
                                                             XPathNSResolver* resolver,
                                                             ExceptionCode& ec)
{
    return XPathExpression::createExpression(expression, resolver, ec);
}

PassRefPtr<XPathNSResolver> XPathEvaluator::createNSResolver(Node* nodeResolver)
{
    return NativeXPathNSResolver::create(nodeResolver);
}

PassRefPtr<XPathResult> XPathEvaluator::evaluate(const String& expression,
                                                 Node* contextNode,
                                                 XPathNSResolver* resolver,
                                                 unsigned short type,
                                                 XPathResult* result,
                                                 ExceptionCode& ec)
{
    if (!isValidContextNode(contextNode)) {
        ec = NOT_SUPPORTED_ERR;
        return 0;
    }

    ec = 0;
    RefPtr<XPathExpression> expr = createExpression(expression, resolver, ec);
    if (ec)
        return 0;
    
    return expr->evaluate(contextNode, type, result, ec);
}

}

#endif // ENABLE(XPATH)
