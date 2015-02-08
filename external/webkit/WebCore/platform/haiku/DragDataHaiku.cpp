

#include "config.h"
#include "DragData.h"

#include "ClipboardHaiku.h"
#include "Document.h"
#include "DocumentFragment.h"
#include "NotImplemented.h"


namespace WebCore {

bool DragData::canSmartReplace() const
{
    notImplemented();
    return false;
}

bool DragData::containsColor() const
{
    notImplemented();
    return false;
}

bool DragData::containsFiles() const
{
    notImplemented();
    return false;
}

void DragData::asFilenames(Vector<String>& result) const
{
    notImplemented();
}

bool DragData::containsPlainText() const
{
    notImplemented();
    return false;
}

String DragData::asPlainText() const
{
    notImplemented();
    return String();
}

Color DragData::asColor() const
{
    notImplemented();
    return Color();
}

WTF::PassRefPtr<Clipboard> DragData::createClipboard(ClipboardAccessPolicy policy) const
{
    return ClipboardHaiku::create(policy, true);
}

bool DragData::containsCompatibleContent() const
{
    return containsColor() || containsURL() || containsPlainText();
}

bool DragData::containsURL() const
{
    notImplemented();
    return false;
}

String DragData::asURL(String* title) const
{
    notImplemented();
    return String();
}

PassRefPtr<DocumentFragment> DragData::asFragment(Document*) const
{
    notImplemented();
    return 0;
}

} // namespace WebCore

