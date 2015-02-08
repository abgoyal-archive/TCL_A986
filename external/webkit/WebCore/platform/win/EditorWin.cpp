

#include "config.h"
#include "Editor.h"
#include "EditorClient.h"

#include "ClipboardWin.h"
#include "Document.h"
#include "Element.h"
#include "htmlediting.h"
#include "TextIterator.h"
#include "visible_units.h"

#include <windows.h>

namespace WebCore {

PassRefPtr<Clipboard> Editor::newGeneralClipboard(ClipboardAccessPolicy policy)
{
    COMPtr<IDataObject> clipboardData;
    if (!SUCCEEDED(OleGetClipboard(&clipboardData)))
        clipboardData = 0;

    return ClipboardWin::create(false, clipboardData.get(), policy);
}

} // namespace WebCore
