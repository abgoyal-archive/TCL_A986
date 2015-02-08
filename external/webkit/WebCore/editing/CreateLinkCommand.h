

#ifndef CreateLinkCommand_h
#define CreateLinkCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class CreateLinkCommand : public CompositeEditCommand {
public:
    static PassRefPtr<CreateLinkCommand> create(Document* document, const String& linkURL)
    {
        return adoptRef(new CreateLinkCommand(document, linkURL));
    }

private:
    CreateLinkCommand(Document*, const String& linkURL);

    virtual void doApply();
    virtual EditAction editingAction() const { return EditActionCreateLink; }

    String m_url;
};

} // namespace WebCore

#endif // CreateLinkCommand_h
