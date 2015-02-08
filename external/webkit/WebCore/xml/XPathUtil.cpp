

#include "config.h"
#include "XPathUtil.h"

#if ENABLE(XPATH)

#include "ContainerNode.h"

namespace WebCore {
namespace XPath {
        
bool isRootDomNode(Node* node)
{
    return node && !node->parentNode();
}

String stringValue(Node* node)
{
    switch (node->nodeType()) {
        case Node::ATTRIBUTE_NODE:
        case Node::PROCESSING_INSTRUCTION_NODE:
        case Node::COMMENT_NODE:
        case Node::TEXT_NODE:
        case Node::CDATA_SECTION_NODE:
        case Node::XPATH_NAMESPACE_NODE:
            return node->nodeValue();
        default:
            if (isRootDomNode(node) || node->nodeType() == Node::ELEMENT_NODE) {
                Vector<UChar> result;
                result.reserveCapacity(1024);

                for (Node* n = node->firstChild(); n; n = n->traverseNextNode(node)) {
                    if (n->isTextNode()) {
                        const String& nodeValue = n->nodeValue();
                        result.append(nodeValue.characters(), nodeValue.length());
                    }
                }

                return String::adopt(result);
            }
    }
    
    return String();
}

bool isValidContextNode(Node* node)
{
    if (!node)
        return false;
    switch (node->nodeType()) {
        case Node::ATTRIBUTE_NODE:
        case Node::CDATA_SECTION_NODE:
        case Node::COMMENT_NODE:
        case Node::DOCUMENT_NODE:
        case Node::ELEMENT_NODE:
        case Node::PROCESSING_INSTRUCTION_NODE:
        case Node::XPATH_NAMESPACE_NODE:
            return true;
        case Node::DOCUMENT_FRAGMENT_NODE:
        case Node::DOCUMENT_TYPE_NODE:
        case Node::ENTITY_NODE:
        case Node::ENTITY_REFERENCE_NODE:
        case Node::NOTATION_NODE:
            return false;
        case Node::TEXT_NODE:
            return !(node->parentNode() && node->parentNode()->isAttributeNode());
    }
    ASSERT_NOT_REACHED();
    return false;
}

}
}

#endif // ENABLE(XPATH)
