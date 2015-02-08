

#ifndef AppendNodeCommand_h
#define AppendNodeCommand_h

#include "EditCommand.h"

namespace WebCore {

class AppendNodeCommand : public SimpleEditCommand {
public:
    static PassRefPtr<AppendNodeCommand> create(PassRefPtr<Element> parent, PassRefPtr<Node> node)
    {
        return adoptRef(new AppendNodeCommand(parent, node));
    }

private:
    AppendNodeCommand(PassRefPtr<Element> parent, PassRefPtr<Node> node);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<Element> m_parent;
    RefPtr<Node> m_node;
};

} // namespace WebCore

#endif // AppendNodeCommand_h
