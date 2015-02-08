

#ifndef RemoveNodePreservingChildrenCommand_h
#define RemoveNodePreservingChildrenCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class RemoveNodePreservingChildrenCommand : public CompositeEditCommand {
public:
    static PassRefPtr<RemoveNodePreservingChildrenCommand> create(PassRefPtr<Node> node)
    {
        return adoptRef(new RemoveNodePreservingChildrenCommand(node));
    }

private:
    RemoveNodePreservingChildrenCommand(PassRefPtr<Node>);

    virtual void doApply();

    RefPtr<Node> m_node;
};

} // namespace WebCore

#endif // RemoveNodePreservingChildrenCommand_h
