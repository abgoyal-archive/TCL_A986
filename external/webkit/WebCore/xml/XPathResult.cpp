

#include "config.h"
#include "XPathResult.h"

#if ENABLE(XPATH)

#include "Document.h"
#include "Node.h"
#include "ExceptionCode.h"
#include "XPathEvaluator.h"
#include "XPathException.h"

namespace WebCore {

using namespace XPath;

XPathResult::XPathResult(Document* document, const Value& value)
    : m_value(value)
{
    switch (m_value.type()) {
        case Value::BooleanValue:
            m_resultType = BOOLEAN_TYPE;
            return;
        case Value::NumberValue:
            m_resultType = NUMBER_TYPE;
            return;
        case Value::StringValue:
            m_resultType = STRING_TYPE;
            return;
        case Value::NodeSetValue:
            m_resultType = UNORDERED_NODE_ITERATOR_TYPE;
            m_nodeSetPosition = 0;
            m_nodeSet = m_value.toNodeSet();
            m_document = document;
            m_domTreeVersion = document->domTreeVersion();
            return;
    }
    ASSERT_NOT_REACHED();
}

XPathResult::~XPathResult()
{
}

void XPathResult::convertTo(unsigned short type, ExceptionCode& ec)
{
    switch (type) {
        case ANY_TYPE:
            break;
        case NUMBER_TYPE:
            m_resultType = type;
            m_value = m_value.toNumber();
            break;
        case STRING_TYPE:
            m_resultType = type;
            m_value = m_value.toString();
            break;
        case BOOLEAN_TYPE:
            m_resultType = type;
            m_value = m_value.toBoolean();
            break;
        case UNORDERED_NODE_ITERATOR_TYPE:
        case UNORDERED_NODE_SNAPSHOT_TYPE:
        case ANY_UNORDERED_NODE_TYPE:
        case FIRST_ORDERED_NODE_TYPE: // This is correct - singleNodeValue() will take care of ordering.
            if (!m_value.isNodeSet()) {
                ec = XPathException::TYPE_ERR;
                return;
            }
            m_resultType = type;
            break;
        case ORDERED_NODE_ITERATOR_TYPE:
            if (!m_value.isNodeSet()) {
                ec = XPathException::TYPE_ERR;
                return;
            }
            m_nodeSet.sort();
            m_resultType = type;
            break;
        case ORDERED_NODE_SNAPSHOT_TYPE:
            if (!m_value.isNodeSet()) {
                ec = XPathException::TYPE_ERR;
                return;
            }
            m_value.toNodeSet().sort();
            m_resultType = type;
            break;
    }
}

unsigned short XPathResult::resultType() const
{
    return m_resultType;
}

double XPathResult::numberValue(ExceptionCode& ec) const
{
    if (resultType() != NUMBER_TYPE) {
        ec = XPathException::TYPE_ERR;
        return 0.0;
    }
    return m_value.toNumber();
}

String XPathResult::stringValue(ExceptionCode& ec) const
{
    if (resultType() != STRING_TYPE) {
        ec = XPathException::TYPE_ERR;
        return String();
    }
    return m_value.toString();
}

bool XPathResult::booleanValue(ExceptionCode& ec) const
{
    if (resultType() != BOOLEAN_TYPE) {
        ec = XPathException::TYPE_ERR;
        return false;
    }
    return m_value.toBoolean();
}

Node* XPathResult::singleNodeValue(ExceptionCode& ec) const
{
    if (resultType() != ANY_UNORDERED_NODE_TYPE && resultType() != FIRST_ORDERED_NODE_TYPE) {
        ec = XPathException::TYPE_ERR;
        return 0;
    }
  
    const NodeSet& nodes = m_value.toNodeSet();
    if (resultType() == FIRST_ORDERED_NODE_TYPE)
        return nodes.firstNode();
    else
        return nodes.anyNode();
}

bool XPathResult::invalidIteratorState() const
{
    if (resultType() != UNORDERED_NODE_ITERATOR_TYPE && resultType() != ORDERED_NODE_ITERATOR_TYPE)
        return false;

    ASSERT(m_document);
    return m_document->domTreeVersion() != m_domTreeVersion;
}

unsigned long XPathResult::snapshotLength(ExceptionCode& ec) const
{
    if (resultType() != UNORDERED_NODE_SNAPSHOT_TYPE && resultType() != ORDERED_NODE_SNAPSHOT_TYPE) {
        ec = XPathException::TYPE_ERR;
        return 0;
    }

    return m_value.toNodeSet().size();
}

Node* XPathResult::iterateNext(ExceptionCode& ec)
{
    if (resultType() != UNORDERED_NODE_ITERATOR_TYPE && resultType() != ORDERED_NODE_ITERATOR_TYPE) {
        ec = XPathException::TYPE_ERR;
        return 0;
    }
    
    if (invalidIteratorState()) {
        ec = INVALID_STATE_ERR;
        return 0;
    }
    
    if (m_nodeSetPosition + 1 > m_nodeSet.size())
        return 0;

    Node* node = m_nodeSet[m_nodeSetPosition];
    
    m_nodeSetPosition++;

    return node;
}

Node* XPathResult::snapshotItem(unsigned long index, ExceptionCode& ec)
{
    if (resultType() != UNORDERED_NODE_SNAPSHOT_TYPE && resultType() != ORDERED_NODE_SNAPSHOT_TYPE) {
        ec = XPathException::TYPE_ERR;
        return 0;
    }
    
    const NodeSet& nodes = m_value.toNodeSet();
    if (index >= nodes.size())
        return 0;
    
    return nodes[index];
}

}

#endif // ENABLE(XPATH)
