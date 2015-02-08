

#ifndef DocumentFragment_h
#define DocumentFragment_h

#include "ContainerNode.h"

namespace WebCore {

class DocumentFragment : public ContainerNode {
public:
    static PassRefPtr<DocumentFragment> create(Document*);

private:
    DocumentFragment(Document*);

    virtual String nodeName() const;
    virtual NodeType nodeType() const;
    virtual PassRefPtr<Node> cloneNode(bool deep);
    virtual bool childTypeAllowed(NodeType);
};

} //namespace

#endif
