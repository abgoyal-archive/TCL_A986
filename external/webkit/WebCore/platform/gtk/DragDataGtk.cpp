

#include "config.h"
#include "DragData.h"

#include "Clipboard.h"
#include "Document.h"
#include "DocumentFragment.h"

namespace WebCore {

bool DragData::canSmartReplace() const
{
    return false;
}

bool DragData::containsColor() const
{
    return false;
}

bool DragData::containsFiles() const
{
    return false;
}

void DragData::asFilenames(Vector<String>& result) const
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

Color DragData::asColor() const
{
    return Color();
}

PassRefPtr<Clipboard> DragData::createClipboard(ClipboardAccessPolicy) const
{
    return 0;
}

bool DragData::containsCompatibleContent() const
{
    return false;
}

bool DragData::containsURL() const
{
    return false;
}

String DragData::asURL(String* title) const
{
    return String();
}


PassRefPtr<DocumentFragment> DragData::asFragment(Document*) const
{
    return 0;
}

}
