

#include "config.h"
#include "CreateLinkCommand.h"
#include "htmlediting.h"
#include "Text.h"

#include "HTMLAnchorElement.h"

namespace WebCore {

CreateLinkCommand::CreateLinkCommand(Document* document, const String& url) 
    : CompositeEditCommand(document)
{
    m_url = url;
}

void CreateLinkCommand::doApply()
{
    if (endingSelection().isNone())
        return;
        
    RefPtr<HTMLAnchorElement> anchorElement = HTMLAnchorElement::create(document());
    anchorElement->setHref(m_url);
    
    if (endingSelection().isRange()) {
        pushPartiallySelectedAnchorElementsDown();
        applyStyledElement(anchorElement.get());
    } else {
        insertNodeAt(anchorElement.get(), endingSelection().start());
        RefPtr<Text> textNode = Text::create(document(), m_url);
        appendNode(textNode.get(), anchorElement.get());
        setEndingSelection(VisibleSelection(positionInParentBeforeNode(anchorElement.get()), positionInParentAfterNode(anchorElement.get()), DOWNSTREAM));
    }
}

}
