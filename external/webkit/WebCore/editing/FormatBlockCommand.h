

#ifndef FormatBlockCommand_h
#define FormatBlockCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class FormatBlockCommand : public CompositeEditCommand {
public:
    static PassRefPtr<FormatBlockCommand> create(Document* document, const AtomicString& tagName)
    {
        return adoptRef(new FormatBlockCommand(document, tagName));
    }

private:
    FormatBlockCommand(Document*, const AtomicString& tagName);

    virtual void doApply();
    virtual EditAction editingAction() const { return EditActionFormatBlock; }

    bool modifyRange();
    AtomicString m_tagName;
};

} // namespace WebCore

#endif // FormatBlockCommand_h
