

#include "config.h"
#include "Editor.h"

#include "ClipboardWx.h"
#include "NotImplemented.h"

namespace WebCore {

PassRefPtr<Clipboard> Editor::newGeneralClipboard(ClipboardAccessPolicy policy) 
{ 
    return ClipboardWx::create(policy, true);
}

void Editor::showColorPanel()
{
    notImplemented();
}

void Editor::showFontPanel()
{
    notImplemented();
}

void Editor::showStylesPanel()
{
    notImplemented();
}

}
