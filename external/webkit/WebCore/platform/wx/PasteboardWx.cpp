

#include "config.h"

#include "Pasteboard.h"
#include "DocumentFragment.h"
#include "Editor.h"
#include "Frame.h"
#include "KURL.h"
#include "markup.h"
#include "NotImplemented.h"
#include "PlatformString.h"

#include <wx/defs.h>
#include <wx/dataobj.h>
#include <wx/clipbrd.h>

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
    if (wxTheClipboard->Open()) {
        wxTheClipboard->SetData( new wxTextDataObject(frame->selectedText()) );
        wxTheClipboard->Close();
    }
}

void Pasteboard::writePlainText(const String& text)
{
    if (wxTheClipboard->Open()) {
        wxTheClipboard->SetData( new wxTextDataObject(text) );
        wxTheClipboard->Close();
    }
}

bool Pasteboard::canSmartReplace()
{
    notImplemented();
    return false;
}

String Pasteboard::plainText(Frame* frame)
{
    notImplemented();
    return String();
}

PassRefPtr<DocumentFragment> Pasteboard::documentFragment(Frame* frame, PassRefPtr<Range> context,
                                                          bool allowPlainText, bool& chosePlainText)
{
    RefPtr<DocumentFragment> fragment = 0;
    if (wxTheClipboard->Open()) {
        if (allowPlainText && wxTheClipboard->IsSupported( wxDF_TEXT )) {
            wxTextDataObject data;
            wxTheClipboard->GetData( data );
            chosePlainText = true;
            fragment = createFragmentFromText(context.get(), data.GetText());
        }
        wxTheClipboard->Close();
    }
    if (fragment)
        return fragment.release();
    
    return 0;
}

void Pasteboard::writeURL(const KURL& url, const String&, Frame*)
{
    if (wxTheClipboard->Open()) {
        wxTheClipboard->SetData( new wxTextDataObject( url.string() ) );
        wxTheClipboard->Close();
    }
}

void Pasteboard::clear()
{
    wxTheClipboard->Clear();
}

void Pasteboard::writeImage(Node*, const KURL&, const String& title)
{
    notImplemented();
}
    
}
