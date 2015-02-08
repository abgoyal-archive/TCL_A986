

#include "config.h"
#include "DocumentType.h"

#include "DOMImplementation.h"
#include "Document.h"
#include "NamedNodeMap.h"

namespace WebCore {

DocumentType::DocumentType(Document* document, const String& name, const String& publicId, const String& systemId)
    : Node(document, CreateOther)
    , m_name(name)
    , m_publicId(publicId)
    , m_systemId(systemId)
{
}

KURL DocumentType::baseURI() const
{
    return KURL();
}

String DocumentType::nodeName() const
{
    return name();
}

Node::NodeType DocumentType::nodeType() const
{
    return DOCUMENT_TYPE_NODE;
}

PassRefPtr<Node> DocumentType::cloneNode(bool /*deep*/)
{
    return create(document(), m_name, m_publicId, m_systemId);
}

void DocumentType::insertedIntoDocument()
{
    // Our document node can be null if we were created by a DOMImplementation.  We use the parent() instead.
    ASSERT(parent() && parent()->isDocumentNode());
    if (parent() && parent()->isDocumentNode()) {
        Document* doc = static_cast<Document*>(parent());
        if (!doc->doctype())
            doc->setDocType(this);
    }
    Node::insertedIntoDocument();
}

void DocumentType::removedFromDocument()
{
    if (document() && document()->doctype() == this)
        document()->setDocType(0);
    Node::removedFromDocument();
}

}
