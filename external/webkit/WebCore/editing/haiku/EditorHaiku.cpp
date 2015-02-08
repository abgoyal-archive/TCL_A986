

#include "config.h"
#include "Editor.h"

#include "Clipboard.h"
#include "ClipboardAccessPolicy.h"
#include "ClipboardHaiku.h"


namespace WebCore {

PassRefPtr<Clipboard> Editor::newGeneralClipboard(ClipboardAccessPolicy policy)
{
    return ClipboardHaiku::create(policy, false);
}

} // namespace WebCore

