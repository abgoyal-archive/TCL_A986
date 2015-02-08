

#ifndef SplitTextNodeContainingElementCommand_h
#define SplitTextNodeContainingElementCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class SplitTextNodeContainingElementCommand : public CompositeEditCommand {
public:
    static PassRefPtr<SplitTextNodeContainingElementCommand> create(PassRefPtr<Text> node, int offset)
    {
        return adoptRef(new SplitTextNodeContainingElementCommand(node, offset));
    }

private:
    SplitTextNodeContainingElementCommand(PassRefPtr<Text>, int offset);

    virtual void doApply();

    RefPtr<Text> m_text;
    int m_offset;
};

} // namespace WebCore

#endif // SplitTextNodeContainingElementCommand_h
