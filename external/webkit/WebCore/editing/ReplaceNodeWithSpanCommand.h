

#ifndef ReplaceNodeWithSpanCommand_h
#define ReplaceNodeWithSpanCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class HTMLElement;

// More accurately, this is ReplaceNodeWithSpanPreservingChildrenAndAttributesCommand
class ReplaceNodeWithSpanCommand : public CompositeEditCommand {
public:
    static PassRefPtr<ReplaceNodeWithSpanCommand> create(PassRefPtr<Node> node)
    {
        return adoptRef(new ReplaceNodeWithSpanCommand(node));
    }

    HTMLElement* spanElement() { return m_spanElement.get(); }

private:
    ReplaceNodeWithSpanCommand(PassRefPtr<Node>);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<Node> m_node;
    RefPtr<HTMLElement> m_spanElement;
};

} // namespace WebCore

#endif // ReplaceNodeWithSpanCommand
