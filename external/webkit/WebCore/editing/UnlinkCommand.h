

#ifndef UnlinkCommand_h
#define UnlinkCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class UnlinkCommand : public CompositeEditCommand {
public:
    static PassRefPtr<UnlinkCommand> create(Document* document)
    {
        return adoptRef(new UnlinkCommand(document));
    }

private:
    UnlinkCommand(Document*);

    virtual void doApply();
    virtual EditAction editingAction() const { return EditActionUnlink; }
};

} // namespace WebCore

#endif // UnlinkCommand_h
