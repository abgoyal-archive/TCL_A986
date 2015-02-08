

#ifndef XPathExpressionNode_h
#define XPathExpressionNode_h

#if ENABLE(XPATH)

#include "StringHash.h"
#include "Node.h"
#include "XPathValue.h"
#include <wtf/HashMap.h>
#include <wtf/Vector.h>

namespace WebCore {

    namespace XPath {
        
        struct EvaluationContext : FastAllocBase {
            RefPtr<Node> node;
            unsigned long size;
            unsigned long position;
            HashMap<String, String> variableBindings;

            bool hadTypeConversionError;
        };

        class ParseNode {
        public:
            virtual ~ParseNode() { }
        };

        class Expression : public ParseNode, public Noncopyable {
        public:
            static EvaluationContext& evaluationContext();

            Expression();
            virtual ~Expression();

            virtual Value evaluate() const = 0;

            void addSubExpression(Expression* expr)
            {
                m_subExpressions.append(expr);
                m_isContextNodeSensitive |= expr->m_isContextNodeSensitive;
                m_isContextPositionSensitive |= expr->m_isContextPositionSensitive;
                m_isContextSizeSensitive |= expr->m_isContextSizeSensitive;
            }

            bool isContextNodeSensitive() const { return m_isContextNodeSensitive; }
            bool isContextPositionSensitive() const { return m_isContextPositionSensitive; }
            bool isContextSizeSensitive() const { return m_isContextSizeSensitive; }
            void setIsContextNodeSensitive(bool value) { m_isContextNodeSensitive = value; }
            void setIsContextPositionSensitive(bool value) { m_isContextPositionSensitive = value; }
            void setIsContextSizeSensitive(bool value) { m_isContextSizeSensitive = value; }

            virtual Value::Type resultType() const = 0;

        protected:
            unsigned subExprCount() const { return m_subExpressions.size(); }
            Expression* subExpr(unsigned i) { return m_subExpressions[i]; }
            const Expression* subExpr(unsigned i) const { return m_subExpressions[i]; }

        private:
            Vector<Expression*> m_subExpressions;

            // Evaluation details that can be used for optimization.
            bool m_isContextNodeSensitive;
            bool m_isContextPositionSensitive;
            bool m_isContextSizeSensitive;
        };

    }

}

#endif // ENABLE(XPATH)

#endif // EXPRESSION_H

