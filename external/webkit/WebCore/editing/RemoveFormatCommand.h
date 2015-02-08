

#ifndef RemoveFormatCommand_h
#define RemoveFormatCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class RemoveFormatCommand : public CompositeEditCommand {
public:
    static PassRefPtr<RemoveFormatCommand> create(Document* document)
    {
        return adoptRef(new RemoveFormatCommand(document));
    }

private:
    RemoveFormatCommand(Document*);

    virtual void doApply();
    virtual EditAction editingAction() const { return EditActionUnspecified; }
};

} // namespace WebCore

#endif // RemoveFormatCommand_h
