

#ifndef BreakBlockquoteCommand_h
#define BreakBlockquoteCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class BreakBlockquoteCommand : public CompositeEditCommand {
public:
    static PassRefPtr<BreakBlockquoteCommand> create(Document* document)
    {
        return adoptRef(new BreakBlockquoteCommand(document));
    }

private:
    BreakBlockquoteCommand(Document*);
    virtual void doApply();
};

} // namespace WebCore

#endif
