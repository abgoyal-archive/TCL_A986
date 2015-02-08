

#include "config.h"
#include "Editor.h"

#include "ChromiumDataObject.h"
#include "ClipboardChromium.h"

namespace WebCore {

PassRefPtr<Clipboard> Editor::newGeneralClipboard(ClipboardAccessPolicy policy)
{
    return ClipboardChromium::create(false, ChromiumDataObject::create(), policy);
}

} // namespace WebCore
