

#ifndef EntityReference_h
#define EntityReference_h

#include "ContainerNode.h"

namespace WebCore {

class EntityReference : public ContainerNode {
public:
    static PassRefPtr<EntityReference> create(Document*, const String& entityName);

private:
    EntityReference(Document*, const String& entityName);

    virtual String nodeName() const;
    virtual NodeType nodeType() const;
    virtual PassRefPtr<Node> cloneNode(bool deep);

    String m_entityName;
};

} //namespace

#endif
