

#ifndef XPathEvaluator_h
#define XPathEvaluator_h

#if ENABLE(XPATH)

#include <wtf/RefCounted.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

    typedef int ExceptionCode;

    class Node;
    class String;
    class XPathExpression;
    class XPathNSResolver;
    class XPathResult;

    class XPathEvaluator : public RefCounted<XPathEvaluator> {
    public:
        static PassRefPtr<XPathEvaluator> create() { return adoptRef(new XPathEvaluator); }
        
        PassRefPtr<XPathExpression> createExpression(const String& expression, XPathNSResolver*, ExceptionCode&);
        PassRefPtr<XPathNSResolver> createNSResolver(Node* nodeResolver);
        PassRefPtr<XPathResult> evaluate(const String& expression, Node* contextNode,
            XPathNSResolver*, unsigned short type, XPathResult*, ExceptionCode&);

    private:
        XPathEvaluator() { }
    };

}

#endif // ENABLE(XPATH)

#endif // XPathEvaluator_h
