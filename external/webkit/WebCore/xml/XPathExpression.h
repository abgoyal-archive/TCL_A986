

#ifndef XPathExpression_h
#define XPathExpression_h

#if ENABLE(XPATH)

#include <wtf/RefCounted.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

    typedef int ExceptionCode;

    class Node;
    class String;
    class XPathNSResolver;
    class XPathResult;

    namespace XPath {
        class Expression;
    }

    class XPathExpression : public RefCounted<XPathExpression> {
    public:
        static PassRefPtr<XPathExpression> create() { return adoptRef(new XPathExpression); }
        ~XPathExpression();
        
        static PassRefPtr<XPathExpression> createExpression(const String& expression, XPathNSResolver*, ExceptionCode&);
        PassRefPtr<XPathResult> evaluate(Node* contextNode, unsigned short type, XPathResult*, ExceptionCode&);
            
    private:
        XPathExpression() { }

        XPath::Expression* m_topExpression;
    };

}

#endif // ENABLE(XPATH)

#endif // XPathExpression_h
