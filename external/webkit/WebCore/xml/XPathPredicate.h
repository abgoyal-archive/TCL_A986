

#ifndef XPathPredicate_h
#define XPathPredicate_h

#if ENABLE(XPATH)

#include "XPathExpressionNode.h"
#include "XPathValue.h"

namespace WebCore {

    namespace XPath {
        
        class Number : public Expression {
        public:
            Number(double);
        private:
            virtual Value evaluate() const;
            virtual Value::Type resultType() const { return Value::NumberValue; }

            Value m_value;
        };

        class StringExpression : public Expression {
        public:
            StringExpression(const String&);
        private:
            virtual Value evaluate() const;
            virtual Value::Type resultType() const { return Value::StringValue; }

            Value m_value;
        };

        class Negative : public Expression {
        private:
            virtual Value evaluate() const;
            virtual Value::Type resultType() const { return Value::NumberValue; }
        };

        class NumericOp : public Expression {
        public:
            enum Opcode {
                OP_Add, OP_Sub, OP_Mul, OP_Div, OP_Mod
            };
            NumericOp(Opcode, Expression* lhs, Expression* rhs);
        private:
            virtual Value evaluate() const;
            virtual Value::Type resultType() const { return Value::NumberValue; }

            Opcode m_opcode;
        };

        class EqTestOp : public Expression {
        public:
            enum Opcode { OP_EQ, OP_NE, OP_GT, OP_LT, OP_GE, OP_LE };
            EqTestOp(Opcode, Expression* lhs, Expression* rhs);
            virtual Value evaluate() const;
        private:
            virtual Value::Type resultType() const { return Value::BooleanValue; }
            bool compare(const Value&, const Value&) const;

            Opcode m_opcode;
        };

        class LogicalOp : public Expression {
        public:
            enum Opcode { OP_And, OP_Or };
            LogicalOp(Opcode, Expression* lhs, Expression* rhs);
        private:
            virtual Value::Type resultType() const { return Value::BooleanValue; }
            bool shortCircuitOn() const;
            virtual Value evaluate() const;

            Opcode m_opcode;
        };

        class Union : public Expression {
        private:
            virtual Value evaluate() const;
            virtual Value::Type resultType() const { return Value::NodeSetValue; }
        };

        class Predicate : public Noncopyable {
        public:
            Predicate(Expression*);
            ~Predicate();
            bool evaluate() const;

            bool isContextPositionSensitive() const { return m_expr->isContextPositionSensitive() || m_expr->resultType() == Value::NumberValue; }
            bool isContextSizeSensitive() const { return m_expr->isContextSizeSensitive(); }

        private:
            Expression* m_expr;
        };

    }

}

#endif // ENABLE(XPATH)

#endif // XPathPredicate_h
