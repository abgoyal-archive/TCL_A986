

#ifndef InsertLineBreakCommand_h
#define InsertLineBreakCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class InsertLineBreakCommand : public CompositeEditCommand {
public:
    static PassRefPtr<InsertLineBreakCommand> create(Document* document)
    {
        return adoptRef(new InsertLineBreakCommand(document));
    }

private:
    InsertLineBreakCommand(Document*);

    virtual void doApply();

    virtual bool preservesTypingStyle() const;

    void insertNodeAfterPosition(Node*, const Position&);
    void insertNodeBeforePosition(Node*, const Position&);
    bool shouldUseBreakElement(const Position&);
};

} // namespace WebCore

#endif // InsertLineBreakCommand_h
