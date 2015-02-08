

#ifndef XPathVariableReference_h
#define XPathVariableReference_h

#if ENABLE(XPATH)

#include "XPathExpressionNode.h"

namespace WebCore {

    namespace XPath {

        // Variable references are not used with XPathEvaluator.
        class VariableReference : public Expression {
        public:
            VariableReference(const String& name);
        private:
            virtual Value evaluate() const;
            virtual Value::Type resultType() const { ASSERT_NOT_REACHED(); return Value::NumberValue; }
            String m_name;
        };

    }
}

#endif // ENABLE(XPATH)

#endif // XPath_VariableReference_H
