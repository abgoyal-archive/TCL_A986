

#ifndef SplitTextNodeCommand_h
#define SplitTextNodeCommand_h

#include "EditCommand.h"

namespace WebCore {

class Text;

class SplitTextNodeCommand : public SimpleEditCommand {
public:
    static PassRefPtr<SplitTextNodeCommand> create(PassRefPtr<Text> node, int offset)
    {
        return adoptRef(new SplitTextNodeCommand(node, offset));
    }

private:
    SplitTextNodeCommand(PassRefPtr<Text>, int offset);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<Text> m_text1;
    RefPtr<Text> m_text2;
    unsigned m_offset;
};

} // namespace WebCore

#endif // SplitTextNodeCommand_h
