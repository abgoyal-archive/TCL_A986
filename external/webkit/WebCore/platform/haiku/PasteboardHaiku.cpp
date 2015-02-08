

#include "config.h"
#include "Pasteboard.h"

#include "DocumentFragment.h"
#include "Editor.h"
#include "Frame.h"
#include "KURL.h"
#include "NotImplemented.h"
#include "markup.h"
#include <support/Locker.h>
#include <Clipboard.h>
#include <Message.h>
#include <String.h>


namespace WebCore {

Pasteboard::Pasteboard()
{
}

Pasteboard* Pasteboard::generalPasteboard()
{
    static Pasteboard* pasteboard = new Pasteboard();
    return pasteboard;
}

void Pasteboard::writeSelection(Range* selectedRange, bool canSmartCopyOrDelete, Frame* frame)
{
    BClipboard clipboard("WebKit");
    if (!clipboard.Lock())
        return;

    clipboard.Clear();
    BMessage* data = clipboard.Data();
    if (!data)
        return;

    data->AddString("text/plain", BString(frame->selectedText()));
    clipboard.Commit();

    clipboard.Unlock();
}

void Pasteboard::writePlainText(const String& text)
{
    BClipboard clipboard("WebKit");
    if (!clipboard.Lock())
        return;

    clipboard.Clear();
    BMessage* data = clipboard.Data();
    if (!data)
        return;

    data->AddString("text/plain", BString(text));
    clipboard.Commit();

    clipboard.Unlock();
}

bool Pasteboard::canSmartReplace()
{
    notImplemented();
    return false;
}

String Pasteboard::plainText(Frame* frame)
{
    BClipboard clipboard("WebKit");
    if (!clipboard.Lock())
        return String();

    BMessage* data = clipboard.Data();
    if (!data)
        return String();

    BString string;
    data->FindString("text/plain", &string);

    clipboard.Unlock();

    return string;
}

PassRefPtr<DocumentFragment> Pasteboard::documentFragment(Frame* frame, PassRefPtr<Range> context,
                                                          bool allowPlainText, bool& chosePlainText)
{
    notImplemented();
    return 0;
}

void Pasteboard::writeURL(const KURL& url, const String&, Frame*)
{
    BClipboard clipboard("WebKit");
    if (!clipboard.Lock())
        return;

    clipboard.Clear();

    BMessage* data = clipboard.Data();
    if (!data)
        return;

    data->AddString("text/plain", url.string());
    clipboard.Commit();

    clipboard.Unlock();
}

void Pasteboard::writeImage(Node*, const KURL&, const String&)
{
    notImplemented();
}

void Pasteboard::clear()
{
    BClipboard clipboard("WebKit");
    if (!clipboard.Lock())
        return;

    clipboard.Clear();
    clipboard.Commit();

    clipboard.Unlock();
}

} // namespace WebCore

