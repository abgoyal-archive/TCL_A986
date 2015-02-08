

#ifndef InsertIntoTextNodeCommand_h
#define InsertIntoTextNodeCommand_h

#include "EditCommand.h"

namespace WebCore {

class Text;

class InsertIntoTextNodeCommand : public SimpleEditCommand {
public:
    static PassRefPtr<InsertIntoTextNodeCommand> create(PassRefPtr<Text> node, unsigned offset, const String& text)
    {
        return adoptRef(new InsertIntoTextNodeCommand(node, offset, text));
    }

private:
    InsertIntoTextNodeCommand(PassRefPtr<Text> node, unsigned offset, const String& text);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<Text> m_node;
    unsigned m_offset;
    String m_text;
};

} // namespace WebCore

#endif // InsertIntoTextNodeCommand_h
