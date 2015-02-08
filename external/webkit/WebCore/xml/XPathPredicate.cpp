

#include "config.h"

#if ENABLE(XPATH)

#include "XPathPredicate.h"

#include "Node.h"
#include "XPathFunctions.h"
#include "XPathUtil.h"
#include "XPathValue.h"
#include <math.h>
#include <wtf/MathExtras.h>

namespace WebCore {
namespace XPath {
        
Number::Number(double value)
    : m_value(value)
{
}

Value Number::evaluate() const
{
    return m_value;
}

StringExpression::StringExpression(const String& value)
    : m_value(value)
{
}

Value StringExpression::evaluate() const
{
    return m_value;
}

Value Negative::evaluate() const
{
    Value p(subExpr(0)->evaluate());
    return -p.toNumber();
}

NumericOp::NumericOp(Opcode opcode, Expression* lhs, Expression* rhs)
    : m_opcode(opcode)
{
    addSubExpression(lhs);
    addSubExpression(rhs);
}

Value NumericOp::evaluate() const
{
    Value lhs(subExpr(0)->evaluate());
    Value rhs(subExpr(1)->evaluate());
    
    double leftVal = lhs.toNumber();
    double rightVal = rhs.toNumber();

    switch (m_opcode) {
        case OP_Add:
            return leftVal + rightVal;
        case OP_Sub:
            return leftVal - rightVal;
        case OP_Mul:
            return leftVal * rightVal;
        case OP_Div:
            return leftVal / rightVal;
        case OP_Mod:
            return fmod(leftVal, rightVal);
    }
    ASSERT_NOT_REACHED();
    return 0.0;
}

EqTestOp::EqTestOp(Opcode opcode, Expression* lhs, Expression* rhs)
    : m_opcode(opcode)
{
    addSubExpression(lhs);
    addSubExpression(rhs);
}

bool EqTestOp::compare(const Value& lhs, const Value& rhs) const
{
    if (lhs.isNodeSet()) {
        const NodeSet& lhsSet = lhs.toNodeSet();
        if (rhs.isNodeSet()) {
            // If both objects to be compared are node-sets, then the comparison will be true if and only if
            // there is a node in the first node-set and a node in the second node-set such that the result of
            // performing the comparison on the string-values of the two nodes is true.
            const NodeSet& rhsSet = rhs.toNodeSet();
            for (unsigned lindex = 0; lindex < lhsSet.size(); ++lindex)
                for (unsigned rindex = 0; rindex < rhsSet.size(); ++rindex)
                    if (compare(stringValue(lhsSet[lindex]), stringValue(rhsSet[rindex])))
                        return true;
            return false;
        }
        if (rhs.isNumber()) {
            // If one object to be compared is a node-set and the other is a number, then the comparison will be true
            // if and only if there is a node in the node-set such that the result of performing the comparison on the number
            // to be compared and on the result of converting the string-value of that node to a number using the number function is true.
            for (unsigned lindex = 0; lindex < lhsSet.size(); ++lindex)
                if (compare(Value(stringValue(lhsSet[lindex])).toNumber(), rhs))
                    return true;
            return false;
        }
        if (rhs.isString()) {
            // If one object to be compared is a node-set and the other is a string, then the comparison will be true
            // if and only if there is a node in the node-set such that the result of performing the comparison on
            // the string-value of the node and the other string is true.
            for (unsigned lindex = 0; lindex < lhsSet.size(); ++lindex)
                if (compare(stringValue(lhsSet[lindex]), rhs))
                    return true;
            return false;
        }
        if (rhs.isBoolean()) {
            // If one object to be compared is a node-set and the other is a boolean, then the comparison will be true
            // if and only if the result of performing the comparison on the boolean and on the result of converting
            // the node-set to a boolean using the boolean function is true.
            return compare(lhs.toBoolean(), rhs);
        }
        ASSERT(0);
    }
    if (rhs.isNodeSet()) {
        const NodeSet& rhsSet = rhs.toNodeSet();
        if (lhs.isNumber()) {
            for (unsigned rindex = 0; rindex < rhsSet.size(); ++rindex)
                if (compare(lhs, Value(stringValue(rhsSet[rindex])).toNumber()))
                    return true;
            return false;
        }
        if (lhs.isString()) {
            for (unsigned rindex = 0; rindex < rhsSet.size(); ++rindex)
                if (compare(lhs, stringValue(rhsSet[rindex])))
                    return true;
            return false;
        }
        if (lhs.isBoolean())
            return compare(lhs, rhs.toBoolean());
        ASSERT(0);
    }
    
    // Neither side is a NodeSet.
    switch (m_opcode) {
        case OP_EQ:
        case OP_NE:
            bool equal;
            if (lhs.isBoolean() || rhs.isBoolean())
                equal = lhs.toBoolean() == rhs.toBoolean();
            else if (lhs.isNumber() || rhs.isNumber())
                equal = lhs.toNumber() == rhs.toNumber();
            else
                equal = lhs.toString() == rhs.toString();

            if (m_opcode == OP_EQ)
                return equal;
            return !equal;
        case OP_GT:
            return lhs.toNumber() > rhs.toNumber();
        case OP_GE:
            return lhs.toNumber() >= rhs.toNumber();
        case OP_LT:
            return lhs.toNumber() < rhs.toNumber();
        case OP_LE:
            return lhs.toNumber() <= rhs.toNumber();
    }
    ASSERT(0);
    return false;
}

Value EqTestOp::evaluate() const
{
    Value lhs(subExpr(0)->evaluate());
    Value rhs(subExpr(1)->evaluate());

    return compare(lhs, rhs);
}

LogicalOp::LogicalOp(Opcode opcode, Expression* lhs, Expression* rhs)
    : m_opcode(opcode)
{
    addSubExpression(lhs);
    addSubExpression(rhs);
}

bool LogicalOp::shortCircuitOn() const
{
    if (m_opcode == OP_And)
        return false; //false and foo

    return true;  //true or bar
}

Value LogicalOp::evaluate() const
{
    Value lhs(subExpr(0)->evaluate());

    // This is not only an optimization, http://www.w3.org/TR/xpath
    // dictates that we must do short-circuit evaluation
    bool lhsBool = lhs.toBoolean();
    if (lhsBool == shortCircuitOn())
        return lhsBool;

    return subExpr(1)->evaluate().toBoolean();
}

Value Union::evaluate() const
{
    Value lhsResult = subExpr(0)->evaluate();
    Value rhs = subExpr(1)->evaluate();
    
    NodeSet& resultSet = lhsResult.modifiableNodeSet();
    const NodeSet& rhsNodes = rhs.toNodeSet();
    
    HashSet<Node*> nodes;
    for (size_t i = 0; i < resultSet.size(); ++i)
        nodes.add(resultSet[i]);
    
    for (size_t i = 0; i < rhsNodes.size(); ++i) {
        Node* node = rhsNodes[i];
        if (nodes.add(node).second)
            resultSet.append(node);
    }

    // It is also possible to use merge sort to avoid making the result unsorted;
    // but this would waste the time in cases when order is not important.
    resultSet.markSorted(false);
    return lhsResult;
}

Predicate::Predicate(Expression* expr)
    : m_expr(expr)
{
}

Predicate::~Predicate()
{
    delete m_expr;
}

bool Predicate::evaluate() const
{
    ASSERT(m_expr != 0);

    Value result(m_expr->evaluate());

    // foo[3] means foo[position()=3]
    if (result.isNumber())
        return EqTestOp(EqTestOp::OP_EQ, createFunction("position"), new Number(result.toNumber())).evaluate().toBoolean();

    return result.toBoolean();
}

}
}

#endif // ENABLE(XPATH)
