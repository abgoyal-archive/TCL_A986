

#ifndef Notation_h
#define Notation_h

#include "ContainerNode.h"

namespace WebCore {

// FIXME: This class is never instantiated. Maybe it should be removed.

class Notation : public ContainerNode {
public:
    const String& publicId() const { return m_publicId; }
    const String& systemId() const { return m_systemId; }

private:
    Notation(Document*, const String& name, const String& publicId, const String& systemId);

    virtual String nodeName() const;
    virtual NodeType nodeType() const;
    virtual PassRefPtr<Node> cloneNode(bool deep);
    virtual bool childTypeAllowed(NodeType);

    String m_name;
    String m_publicId;
    String m_systemId;
};

} //namespace

#endif
