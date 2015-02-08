

#include "config.h"
#include "EditingText.h"

// FIXME: Does this really require a class? Perhaps instead any text node
// inside an editable element could have the "always create a renderer" behavior.

namespace WebCore {

inline EditingText::EditingText(Document* document, const String& data)
    : Text(document, data)
{
}

PassRefPtr<EditingText> EditingText::create(Document* document, const String& data)
{
    return adoptRef(new EditingText(document, data));
}

bool EditingText::rendererIsNeeded(RenderStyle*)
{
    return true;
}

} // namespace WebCore
