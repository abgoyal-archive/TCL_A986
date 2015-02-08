

#include "config.h"
#include "MoveSelectionCommand.h"

#include "DocumentFragment.h"
#include "ReplaceSelectionCommand.h"

namespace WebCore {

MoveSelectionCommand::MoveSelectionCommand(PassRefPtr<DocumentFragment> fragment, const Position& position, bool smartMove) 
    : CompositeEditCommand(position.node()->document()), m_fragment(fragment), m_position(position), m_smartMove(smartMove)
{
    ASSERT(m_fragment);
}

void MoveSelectionCommand::doApply()
{
    VisibleSelection selection = endingSelection();
    ASSERT(selection.isRange());

    Position pos = m_position;
    if (pos.isNull())
        return;
        
    // Update the position otherwise it may become invalid after the selection is deleted.
    Node *positionNode = m_position.node();
    int positionOffset = m_position.deprecatedEditingOffset();
    Position selectionEnd = selection.end();
    int selectionEndOffset = selectionEnd.deprecatedEditingOffset();
    if (selectionEnd.node() == positionNode && selectionEndOffset < positionOffset) {
        positionOffset -= selectionEndOffset;
        Position selectionStart = selection.start();
        if (selectionStart.node() == positionNode) {
            positionOffset += selectionStart.deprecatedEditingOffset();
        }
        pos = Position(positionNode, positionOffset);
    }

    deleteSelection(m_smartMove);

    // If the node for the destination has been removed as a result of the deletion,
    // set the destination to the ending point after the deletion.
    // Fixes: <rdar://problem/3910425> REGRESSION (Mail): Crash in ReplaceSelectionCommand; 
    //        selection is empty, leading to null deref
    if (!pos.node()->inDocument())
        pos = endingSelection().start();

    setEndingSelection(VisibleSelection(pos, endingSelection().affinity()));
    if (!positionNode->inDocument()) {
        // Document was modified out from under us.
        return;
    }
    applyCommandToComposite(ReplaceSelectionCommand::create(positionNode->document(), m_fragment, true, m_smartMove));
}

EditAction MoveSelectionCommand::editingAction() const
{
    return EditActionDrag;
}

} // namespace WebCore
