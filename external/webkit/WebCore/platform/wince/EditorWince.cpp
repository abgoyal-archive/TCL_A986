

#include "config.h"

#include "Editor.h"

#include "ClipboardWince.h"
#include "Document.h"
#include "EditorClient.h"
#include "Element.h"
#include "HtmlEditing.h"
#include "TextIterator.h"
#include "visible_units.h"

#include <windows.h>
#define _SYS_GUID_OPERATORS_

namespace WebCore {

PassRefPtr<Clipboard> Editor::newGeneralClipboard(ClipboardAccessPolicy policy)
{
    return adoptRef(new ClipboardWince(policy, false));
}

} // namespace WebCore
