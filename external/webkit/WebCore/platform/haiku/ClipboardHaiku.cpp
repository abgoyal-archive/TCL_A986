

#include "config.h"
#include "ClipboardHaiku.h"

#include "FileList.h"
#include "IntPoint.h"
#include "NotImplemented.h"
#include "PlatformString.h"
#include "StringHash.h"

#include <support/Locker.h>
#include <app/Clipboard.h>
#include <Message.h>
#include <String.h>
#include <wtf/HashTable.h>


namespace WebCore {

ClipboardHaiku::ClipboardHaiku(ClipboardAccessPolicy policy, bool forDragging)
    : Clipboard(policy, forDragging)
{
}

void ClipboardHaiku::clearData(const String& type)
{
    if (be_clipboard->Lock()) {
        BMessage* data = be_clipboard->Data();

        if (data) {
            data->RemoveName(BString(type).String());
            be_clipboard->Commit();
        }

        be_clipboard->Unlock();
    }
}

void ClipboardHaiku::clearAllData()
{
    if (be_clipboard->Lock()) {
        be_clipboard->Clear();
        be_clipboard->Commit();
        be_clipboard->Unlock();
    }
}

String ClipboardHaiku::getData(const String& type, bool& success) const
{
    BString result;
    success = false;

    if (be_clipboard->Lock()) {
        BMessage* data = be_clipboard->Data();

        if (data)
            if (data->FindString(BString(type).String(), &result) == B_OK)
                success = true;

        be_clipboard->Unlock();
    }

    return result;
}

bool ClipboardHaiku::setData(const String& type, const String& data)
{
    bool result = false;

    if (be_clipboard->Lock()) {
        BMessage* bdata = be_clipboard->Data();

        if (bdata) {
            bdata->RemoveName(BString(type).String());

            if (bdata->AddString(BString(type).String(), BString(data)) == B_OK)
                result = true;
        }

        be_clipboard->Commit();
        be_clipboard->Unlock();
    }

    return result;
}

// Extensions beyond IE's API.
HashSet<String> ClipboardHaiku::types() const
{
    HashSet<String> result;

    if (be_clipboard->Lock()) {
        BMessage* data = be_clipboard->Data();

        if (data) {
            char* name;
            uint32 type;
            int32 count;

            for (int32 i = 0; data->GetInfo(B_ANY_TYPE, i, &name, &type, &count) == B_OK; i++)
                result.add(name);
        }

        be_clipboard->Unlock();
    }

    return result;
}

PassRefPtr<FileList> ClipboardHaiku::files() const
{
    notImplemented();
    return 0;
}

IntPoint ClipboardHaiku::dragLocation() const
{
    notImplemented();
    return IntPoint(0, 0);
}

CachedImage* ClipboardHaiku::dragImage() const
{
    notImplemented();
    return 0;
}

void ClipboardHaiku::setDragImage(CachedImage*, const IntPoint&)
{
    notImplemented();
}

Node* ClipboardHaiku::dragImageElement()
{
    notImplemented();
    return 0;
}

void ClipboardHaiku::setDragImageElement(Node*, const IntPoint&)
{
    notImplemented();
}

DragImageRef ClipboardHaiku::createDragImage(IntPoint& dragLocation) const
{
    notImplemented();
    return 0;
}

void ClipboardHaiku::declareAndWriteDragImage(Element*, const KURL&, const String&, Frame*)
{
    notImplemented();
}

void ClipboardHaiku::writeURL(const KURL&, const String&, Frame*)
{
    notImplemented();
}

void ClipboardHaiku::writeRange(Range*, Frame*)
{
    notImplemented();
}

void ClipboardHaiku::writePlainText(const String&)
{
    notImplemented();
}

bool ClipboardHaiku::hasData()
{
    bool result = false;

    if (be_clipboard->Lock()) {
        BMessage* data = be_clipboard->Data();

        if (data)
            result = !data->IsEmpty();

        be_clipboard->Unlock();
    }

    return result;
}

} // namespace WebCore

