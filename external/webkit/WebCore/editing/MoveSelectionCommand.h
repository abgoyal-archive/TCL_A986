

#ifndef MoveSelectionCommand_h
#define MoveSelectionCommand_h

#include "CompositeEditCommand.h"

namespace WebCore {

class DocumentFragment;

class MoveSelectionCommand : public CompositeEditCommand {
public:
    static PassRefPtr<MoveSelectionCommand> create(PassRefPtr<DocumentFragment> fragment, const Position& position, bool smartMove = false)
    {
        return adoptRef(new MoveSelectionCommand(fragment, position, smartMove));
    }

private:
    MoveSelectionCommand(PassRefPtr<DocumentFragment>, const Position&, bool smartMove);

    virtual void doApply();
    virtual EditAction editingAction() const;
    
    RefPtr<DocumentFragment> m_fragment;
    Position m_position;
    bool m_smartMove;
};

} // namespace WebCore

#endif // MoveSelectionCommand_h
