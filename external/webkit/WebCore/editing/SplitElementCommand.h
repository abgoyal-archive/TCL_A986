

#ifndef SplitElementCommand_h
#define SplitElementCommand_h

#include "EditCommand.h"

namespace WebCore {

class SplitElementCommand : public SimpleEditCommand {
public:
    static PassRefPtr<SplitElementCommand> create(PassRefPtr<Element> element, PassRefPtr<Node> splitPointChild)
    {
        return adoptRef(new SplitElementCommand(element, splitPointChild));
    }

private:
    SplitElementCommand(PassRefPtr<Element>, PassRefPtr<Node> splitPointChild);

    virtual void doApply();
    virtual void doUnapply();
    virtual void doReapply();
    void executeApply();

    RefPtr<Element> m_element1;
    RefPtr<Element> m_element2;
    RefPtr<Node> m_atChild;
};

} // namespace WebCore

#endif // SplitElementCommand_h
