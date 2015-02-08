

#include "config.h"
#include "Editor.h"

#include "ClipboardAccessPolicy.h"
#include "ClipboardQt.h"
#include "Document.h"
#include "Element.h"
#include "VisibleSelection.h"
#include "SelectionController.h"
#include "TextIterator.h"
#include "htmlediting.h"
#include "visible_units.h"

namespace WebCore {

PassRefPtr<Clipboard> Editor::newGeneralClipboard(ClipboardAccessPolicy policy)
{
    return ClipboardQt::create(policy);
}

} // namespace WebCore
