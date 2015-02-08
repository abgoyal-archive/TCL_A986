

#ifndef DocumentType_h
#define DocumentType_h

#include "Node.h"

namespace WebCore {

class NamedNodeMap;

class DocumentType : public Node {
public:
    static PassRefPtr<DocumentType> create(Document* document, const String& name, const String& publicId, const String& systemId)
    {
        return adoptRef(new DocumentType(document, name, publicId, systemId));
    }

    NamedNodeMap* entities() const { return m_entities.get(); }
    NamedNodeMap* notations() const { return m_notations.get(); }

    const String& name() const { return m_name; }
    const String& publicId() const { return m_publicId; }
    const String& systemId() const { return m_systemId; }
    const String& internalSubset() const { return m_subset; }

private:
    DocumentType(Document*, const String& name, const String& publicId, const String& systemId);

    virtual KURL baseURI() const;
    virtual String nodeName() const;
    virtual NodeType nodeType() const;
    virtual PassRefPtr<Node> cloneNode(bool deep);

    virtual void insertedIntoDocument();
    virtual void removedFromDocument();

    RefPtr<NamedNodeMap> m_entities;
    RefPtr<NamedNodeMap> m_notations;

    String m_name;
    String m_publicId;
    String m_systemId;
    String m_subset;
};

} // namespace WebCore

#endif
