

#ifndef InsertNodeBeforeCommand_h
#define InsertNodeBeforeCommand_h

#include "EditCommand.h"

namespace WebCore {

class InsertNodeBeforeCommand : public SimpleEditCommand {
public:
    static PassRefPtr<InsertNodeBeforeCommand> create(PassRefPtr<Node> childToInsert, PassRefPtr<Node> childToInsertBefore)
    {
        return adoptRef(new InsertNodeBeforeCommand(childToInsert, childToInsertBefore));
    }

private:
    InsertNodeBeforeCommand(PassRefPtr<Node> childToInsert, PassRefPtr<Node> childToInsertBefore);

    virtual void doApply();
    virtual void doUnapply();

    RefPtr<Node> m_insertChild;
    RefPtr<Node> m_refChild; 
};

} // namespace WebCore

#endif // InsertNodeBeforeCommand_h
