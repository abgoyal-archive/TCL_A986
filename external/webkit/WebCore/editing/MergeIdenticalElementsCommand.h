

#ifndef MergeIdenticalElementsCommand_h
#define MergeIdenticalElementsCommand_h

#include "EditCommand.h"

namespace WebCore {

class MergeIdenticalElementsCommand : public SimpleEditCommand {
public:
    static PassRefPtr<MergeIdenticalElementsCommand> create(PassRefPtr<Element> element1, PassRefPtr<Element> element2)
    {
        return adoptRef(new MergeIdenticalElementsCommand(element1, element2));
    }

private:
    MergeIdenticalElementsCommand(PassRefPtr<Element>, PassRefPtr<Element>);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<Element> m_element1;
    RefPtr<Element> m_element2;
    RefPtr<Node> m_atChild;
};

} // namespace WebCore

#endif // MergeIdenticalElementsCommand_h
