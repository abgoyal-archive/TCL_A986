

#include "config.h"
#include "ClipboardWx.h"

#include "FileList.h"
#include "HashTable.h"
#include "IntPoint.h"
#include "NotImplemented.h"
#include "Pasteboard.h"
#include "PlatformString.h"
#include "StringHash.h"


namespace WebCore {
    
ClipboardWx::ClipboardWx(ClipboardAccessPolicy policy, bool forDragging) 
    : Clipboard(policy, forDragging)
{
}

void ClipboardWx::clearData(const String& type)
{
    notImplemented();
}

void ClipboardWx::clearAllData() 
{
    Pasteboard::generalPasteboard()->clear();
}

String ClipboardWx::getData(const String& type, bool& success) const 
{
    notImplemented();
    return ""; 
}

bool ClipboardWx::setData(const String& type, const String& data) 
{
    notImplemented();
    return false;
}

// extensions beyond IE's API
HashSet<String> ClipboardWx::types() const 
{
    notImplemented();
    HashSet<String> result;
    return result;
}

PassRefPtr<FileList> ClipboardWx::files() const
{
    notImplemented();
    return 0;
}

IntPoint ClipboardWx::dragLocation() const 
{ 
    notImplemented();
    return IntPoint(0,0);
}

CachedImage* ClipboardWx::dragImage() const 
{
    notImplemented();
    return 0; 
}

void ClipboardWx::setDragImage(CachedImage*, const IntPoint&) 
{
    notImplemented();
}

Node* ClipboardWx::dragImageElement() 
{
    notImplemented();
    return 0; 
}

void ClipboardWx::setDragImageElement(Node*, const IntPoint&)
{
    notImplemented();
}

DragImageRef ClipboardWx::createDragImage(IntPoint& dragLoc) const
{ 
    notImplemented();
    return 0;
}

void ClipboardWx::declareAndWriteDragImage(Element*, const KURL&, const String&, Frame*) 
{
    notImplemented();
}

void ClipboardWx::writeURL(const KURL& url, const String& string, Frame* frame) 
{
    Pasteboard::generalPasteboard()->writeURL(url, string, frame);
}

void ClipboardWx::writeRange(Range*, Frame*) 
{
    notImplemented();
}

bool ClipboardWx::hasData() 
{
    notImplemented();
    return false;
}

void ClipboardWx::writePlainText(const WebCore::String& text)
{
    Pasteboard::generalPasteboard()->writePlainText(text);
}

}
