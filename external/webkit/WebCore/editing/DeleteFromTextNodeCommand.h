

#ifndef DeleteFromTextNodeCommand_h
#define DeleteFromTextNodeCommand_h

#include "EditCommand.h"

namespace WebCore {

class Text;

class DeleteFromTextNodeCommand : public SimpleEditCommand {
public:
    static PassRefPtr<DeleteFromTextNodeCommand> create(PassRefPtr<Text> node, unsigned offset, unsigned count)
    {
        return adoptRef(new DeleteFromTextNodeCommand(node, offset, count));
    }

private:
    DeleteFromTextNodeCommand(PassRefPtr<Text>, unsigned offset, unsigned count);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<Text> m_node;
    unsigned m_offset;
    unsigned m_count;
    String m_text;
};

} // namespace WebCore

#endif // DeleteFromTextNodeCommand_h
