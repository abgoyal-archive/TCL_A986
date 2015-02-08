

#include "config.h"
#include "PlaceholderDocument.h"

#include "CSSStyleSelector.h"
#include "StyleSheetList.h"

namespace WebCore {

void PlaceholderDocument::attach()
{
    ASSERT(!attached());

    // Skipping Document::attach().
    ContainerNode::attach();
}

} // namespace WebCore
