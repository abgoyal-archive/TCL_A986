

#ifndef XPathFunctions_h
#define XPathFunctions_h

#if ENABLE(XPATH)

#include "XPathExpressionNode.h"

namespace WebCore {

    namespace XPath {

        class Function : public Expression {
        public:
            void setArguments(const Vector<Expression*>&);
            void setName(const String& name) { m_name = name; }
        protected:
            Expression* arg(int pos) { return subExpr(pos); }
            const Expression* arg(int pos) const { return subExpr(pos); }
            unsigned int argCount() const { return subExprCount(); }
            String name() const { return m_name; }

        private:
            String m_name;
        };

        Function* createFunction(const String& name, const Vector<Expression*>& args = Vector<Expression*>());

    } // namespace XPath

} // namespace WebCore

#endif // ENABLE(XPATH)

#endif // XPathFunctions_h
