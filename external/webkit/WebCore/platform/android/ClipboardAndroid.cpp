

#include "config.h"
#include "ClipboardAndroid.h"

#include "CachedImage.h"
#include "Element.h"
#include "FileList.h"
#include "Frame.h"
#include "Range.h"

namespace WebCore {

ClipboardAndroid::ClipboardAndroid(ClipboardAccessPolicy policy, bool isForDragging)
    : Clipboard(policy, isForDragging)
{
}

ClipboardAndroid::~ClipboardAndroid()
{
}

void ClipboardAndroid::clearData(const String&)
{
    ASSERT(isForDragging());
}

void ClipboardAndroid::clearAllData()
{
    ASSERT(isForDragging());
}

String ClipboardAndroid::getData(const String&, bool& success) const
{     
    success = false;
    return "";
}

bool ClipboardAndroid::setData(const String&, const String&)
{
    ASSERT(isForDragging());
    return false;
}

// extensions beyond IE's API
HashSet<String> ClipboardAndroid::types() const
{ 
    return HashSet<String>();
}

PassRefPtr<FileList> ClipboardAndroid::files() const
{
    return 0;
}

void ClipboardAndroid::setDragImage(CachedImage*, const IntPoint&)
{
}

void ClipboardAndroid::setDragImageElement(Node*, const IntPoint&)
{
}

DragImageRef ClipboardAndroid::createDragImage(IntPoint&) const
{
    return 0;
}

void ClipboardAndroid::declareAndWriteDragImage(Element*, const KURL&, const String&, Frame*)
{
}

void ClipboardAndroid::writeURL(const KURL&, const String&, Frame*)
{
}

void ClipboardAndroid::writeRange(Range* selectedRange, Frame*)
{
    ASSERT(selectedRange);
}

void ClipboardAndroid::writePlainText(const String&)
{
}

bool ClipboardAndroid::hasData()
{
    return false;
}

} // namespace WebCore
