

#include "config.h"
#include "UnlinkCommand.h"

#include "HTMLAnchorElement.h"

namespace WebCore {

UnlinkCommand::UnlinkCommand(Document* document)
    : CompositeEditCommand(document)
{
}

void UnlinkCommand::doApply()
{
    // FIXME: If a caret is inside a link, we should remove it, but currently we don't.
    if (!endingSelection().isRange())
        return;
        
    pushPartiallySelectedAnchorElementsDown();

    removeStyledElement(HTMLAnchorElement::create(document()));
}

}
