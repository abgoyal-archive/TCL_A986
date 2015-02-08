

#include "config.h"
#include "XMLSerializer.h"

#include "Document.h"
#include "ExceptionCode.h"
#include "markup.h"

namespace WebCore {

String XMLSerializer::serializeToString(Node* node, ExceptionCode& ec)
{
    if (!node)
        return String();

    if (!node->document()) {
        // Due to the fact that DocumentType nodes are created by the DOMImplementation
        // and not the Document, it is possible for it to not have a Document associated
        // with it.  It should be the only type of node where this is possible.
        ASSERT(node->nodeType() == Node::DOCUMENT_TYPE_NODE);

        ec = INVALID_ACCESS_ERR;
        return String();
    }

    return createMarkup(node);
}

} // namespace WebCore
