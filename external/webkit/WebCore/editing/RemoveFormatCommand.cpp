

#include "config.h"
#include "RemoveFormatCommand.h"

#include "CSSComputedStyleDeclaration.h"
#include "CSSMutableStyleDeclaration.h"
#include "Editor.h"
#include "Frame.h"
#include "HTMLNames.h"
#include "VisibleSelection.h"
#include "SelectionController.h"
#include "TextIterator.h"
#include "TypingCommand.h"
#include "ApplyStyleCommand.h"

namespace WebCore {

using namespace HTMLNames;

RemoveFormatCommand::RemoveFormatCommand(Document* document)
    : CompositeEditCommand(document)
{
}

void RemoveFormatCommand::doApply()
{
    Frame* frame = document()->frame();
    
    // Make a plain text string from the selection to remove formatting like tables and lists.
    String string = plainText(frame->selection()->selection().toNormalizedRange().get());

    // Get the default style for this editable root, it's the style that we'll give the
    // content that we're operating on.
    Node* root = frame->selection()->rootEditableElement();
    RefPtr<CSSMutableStyleDeclaration> defaultStyle = editingStyleAtPosition(Position(root, 0));

    // Delete the selected content.
    // FIXME: We should be able to leave this to insertText, but its delete operation
    // doesn't preserve the style we're about to set.
    deleteSelection();
    
    // Delete doesn't remove fully selected lists.
    while (breakOutOfEmptyListItem())
        ;
    
    // If the selection was all formatting (like an empty list) the format-less text will 
    // be empty. Early return since we don't need to do any of the work that follows and
    // to avoid the ASSERT that fires if input(...) is called with an empty String.
    if (string.isEmpty())
        return;
    
    // Insert the content with the default style.
    // See <rdar://problem/5794382> RemoveFormat doesn't always reset text alignment
    frame->setTypingStyle(defaultStyle.get());
    
    inputText(string, true);
}

}
