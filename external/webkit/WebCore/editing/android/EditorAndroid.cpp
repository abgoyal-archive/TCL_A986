

#include "config.h"
#include "Editor.h"

#include "ClipboardAndroid.h"

namespace WebCore {

PassRefPtr<Clipboard> Editor::newGeneralClipboard(ClipboardAccessPolicy policy)
{
    return new ClipboardAndroid(policy, false);
}

} // namespace WebCore
