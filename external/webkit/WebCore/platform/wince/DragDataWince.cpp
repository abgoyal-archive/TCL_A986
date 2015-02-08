

#include "config.h"
#include "DragData.h"

#include "Clipboard.h"
#include "DocumentFragment.h"
#include "PlatformString.h"

namespace WebCore {

PassRefPtr<Clipboard> DragData::createClipboard(ClipboardAccessPolicy policy) const
{
    return 0;
}

bool DragData::containsURL() const
{
    return false;
}

String DragData::asURL(String* title) const
{
    return String();
}

bool DragData::containsFiles() const
{
    return false;
}

void DragData::asFilenames(Vector<String>&) const
{
}

bool DragData::containsPlainText() const
{
    return false;
}

String DragData::asPlainText() const
{
    return String();
}

bool DragData::containsColor() const
{
    return false;
}

bool DragData::canSmartReplace() const
{
    return false;
}

bool DragData::containsCompatibleContent() const
{
    return false;
}

PassRefPtr<DocumentFragment> DragData::asFragment(Document* doc) const
{
     return 0;
}

Color DragData::asColor() const
{
    return Color();
}

}

