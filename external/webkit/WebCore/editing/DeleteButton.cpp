

#include "config.h"
#include "DeleteButton.h"

#include "DeleteButtonController.h"
#include "Document.h"
#include "Editor.h"
#include "Event.h"
#include "EventNames.h"
#include "Frame.h"
#include "HTMLNames.h"

namespace WebCore {

DeleteButton::DeleteButton(Document* document)
    : HTMLImageElement(HTMLNames::imgTag, document)
{
}

void DeleteButton::defaultEventHandler(Event* event)
{
    if (event->isMouseEvent()) {
        if (event->type() == eventNames().clickEvent) {
            document()->frame()->editor()->deleteButtonController()->deleteTarget();
            event->setDefaultHandled();
        }
    }

    HTMLImageElement::defaultEventHandler(event);
}

} // namespace
