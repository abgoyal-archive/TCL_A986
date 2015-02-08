

#include "config.h"
#include "DocumentFragment.h"

namespace WebCore {

inline DocumentFragment::DocumentFragment(Document* document)
    : ContainerNode(document)
{
}

PassRefPtr<DocumentFragment> DocumentFragment::create(Document* document)
{
    return adoptRef(new DocumentFragment(document));
}

String DocumentFragment::nodeName() const
{
    return "#document-fragment";
}

Node::NodeType DocumentFragment::nodeType() const
{
    return DOCUMENT_FRAGMENT_NODE;
}

bool DocumentFragment::childTypeAllowed(NodeType type)
{
    switch (type) {
        case ELEMENT_NODE:
        case PROCESSING_INSTRUCTION_NODE:
        case COMMENT_NODE:
        case TEXT_NODE:
        case CDATA_SECTION_NODE:
        case ENTITY_REFERENCE_NODE:
            return true;
        default:
            return false;
    }
}

PassRefPtr<Node> DocumentFragment::cloneNode(bool deep)
{
    RefPtr<DocumentFragment> clone = create(document());
    if (deep)
        cloneChildNodes(clone.get());
    return clone.release();
}

}
