

#ifndef RemoveNodeCommand_h
#define RemoveNodeCommand_h

#include "EditCommand.h"

namespace WebCore {

class RemoveNodeCommand : public SimpleEditCommand {
public:
    static PassRefPtr<RemoveNodeCommand> create(PassRefPtr<Node> node)
    {
        return adoptRef(new RemoveNodeCommand(node));
    }

private:
    RemoveNodeCommand(PassRefPtr<Node>);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<Node> m_node;
    RefPtr<Node> m_parent;
    RefPtr<Node> m_refChild;    
};

} // namespace WebCore

#endif // RemoveNodeCommand_h
