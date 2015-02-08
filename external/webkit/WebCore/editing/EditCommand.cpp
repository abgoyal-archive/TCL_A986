

#include "config.h"
#include "EditCommand.h"

#include "CompositeEditCommand.h"
#include "CSSComputedStyleDeclaration.h"
#include "CSSMutableStyleDeclaration.h"
#include "DeleteButtonController.h"
#include "Document.h"
#include "Editor.h"
#include "Element.h"
#include "EventNames.h"
#include "Frame.h"
#include "SelectionController.h"
#include "VisiblePosition.h"
#include "htmlediting.h"

namespace WebCore {

EditCommand::EditCommand(Document* document) 
    : m_document(document)
    , m_parent(0)
{
    ASSERT(m_document);
    ASSERT(m_document->frame());
    setStartingSelection(avoidIntersectionWithNode(m_document->frame()->selection()->selection(), m_document->frame()->editor()->deleteButtonController()->containerElement()));
    setEndingSelection(m_startingSelection);
}

EditCommand::~EditCommand()
{
}

void EditCommand::apply()
{
    ASSERT(m_document);
    ASSERT(m_document->frame());
 
    Frame* frame = m_document->frame();
    
    if (!m_parent) {
        if (!endingSelection().isContentRichlyEditable()) {
            switch (editingAction()) {
                case EditActionTyping:
                case EditActionPaste:
                case EditActionDrag:
                case EditActionSetWritingDirection:
                case EditActionCut:
                case EditActionUnspecified:
                    break;
                default:
                    ASSERT_NOT_REACHED();
                    return;
            }
        }
    }
    
    // Changes to the document may have been made since the last editing operation that 
    // require a layout, as in <rdar://problem/5658603>.  Low level operations, like 
    // RemoveNodeCommand, don't require a layout because the high level operations that 
    // use them perform one if one is necessary (like for the creation of VisiblePositions).
    if (!m_parent)
        updateLayout();

    DeleteButtonController* deleteButtonController = frame->editor()->deleteButtonController();
    deleteButtonController->disable();
    doApply();
    deleteButtonController->enable();

    if (!m_parent) {
        updateLayout();
        // Only need to call appliedEditing for top-level commands, and TypingCommands do it on their
        // own (see TypingCommand::typingAddedToOpenCommand).
        if (!isTypingCommand())
            frame->editor()->appliedEditing(this);
    }
}

void EditCommand::unapply()
{
    ASSERT(m_document);
    ASSERT(m_document->frame());
 
    Frame* frame = m_document->frame();
    
    // Changes to the document may have been made since the last editing operation that 
    // require a layout, as in <rdar://problem/5658603>.  Low level operations, like 
    // RemoveNodeCommand, don't require a layout because the high level operations that 
    // use them perform one if one is necessary (like for the creation of VisiblePositions).
    if (!m_parent)
        updateLayout();
    
    DeleteButtonController* deleteButtonController = frame->editor()->deleteButtonController();
    deleteButtonController->disable();
    doUnapply();
    deleteButtonController->enable();

    if (!m_parent) {
        updateLayout();
        frame->editor()->unappliedEditing(this);
    }
}

void EditCommand::reapply()
{
    ASSERT(m_document);
    ASSERT(m_document->frame());
 
    Frame* frame = m_document->frame();
    
    // Changes to the document may have been made since the last editing operation that 
    // require a layout, as in <rdar://problem/5658603>.  Low level operations, like 
    // RemoveNodeCommand, don't require a layout because the high level operations that 
    // use them perform one if one is necessary (like for the creation of VisiblePositions).
    if (!m_parent)
        updateLayout();

    DeleteButtonController* deleteButtonController = frame->editor()->deleteButtonController();
    deleteButtonController->disable();
    doReapply();
    deleteButtonController->enable();

    if (!m_parent) {
        updateLayout();
        frame->editor()->reappliedEditing(this);
    }
}

void EditCommand::doReapply()
{
    doApply();
}

EditAction EditCommand::editingAction() const
{
    return EditActionUnspecified;
}

void EditCommand::setStartingSelection(const VisibleSelection& s)
{
    Element* root = s.rootEditableElement();
    for (EditCommand* cmd = this; ; cmd = cmd->m_parent) {
        cmd->m_startingSelection = s;
        cmd->m_startingRootEditableElement = root;
        if (!cmd->m_parent || cmd->m_parent->isFirstCommand(cmd))
            break;
    }
}

void EditCommand::setEndingSelection(const VisibleSelection &s)
{
    Element* root = s.rootEditableElement();
    for (EditCommand* cmd = this; cmd; cmd = cmd->m_parent) {
        cmd->m_endingSelection = s;
        cmd->m_endingRootEditableElement = root;
    }
}

bool EditCommand::preservesTypingStyle() const
{
    return false;
}

bool EditCommand::isInsertTextCommand() const
{
    return false;
}

bool EditCommand::isTypingCommand() const
{
    return false;
}


void EditCommand::updateLayout() const
{
    document()->updateLayoutIgnorePendingStylesheets();
}

void EditCommand::setParent(CompositeEditCommand* parent)
{
    ASSERT(parent);
    ASSERT(!m_parent);
    m_parent = parent;
    m_startingSelection = parent->m_endingSelection;
    m_endingSelection = parent->m_endingSelection;
    m_startingRootEditableElement = parent->m_endingRootEditableElement;
    m_endingRootEditableElement = parent->m_endingRootEditableElement;
}

void applyCommand(PassRefPtr<EditCommand> command)
{
    command->apply();
}

} // namespace WebCore
