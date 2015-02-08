

#ifndef WebFileChooserParams_h
#define WebFileChooserParams_h

#include "WebFileChooserCompletion.h"
#include "WebString.h"
#include "WebVector.h"

namespace WebKit {

struct WebFileChooserParams {
    // If |multiSelect| is true, the dialog allow to select multiple files.
    bool multiSelect;
    // |title| is a title of a file chooser dialog. It can be an empty string.
    WebString title;
    // |initialValue| is a filename which the dialog should select by default.
    // It can be an empty string.
    WebString initialValue;
    // |acceptTypes| has a comma-separated MIME types such as "audio/*,text/plain".
    // The dialog may restrict selectable files to the specified MIME types.
    // This value comes from an 'accept' attribute value of an INPUT element.
    // So it might be a wrong formatted string.
    WebString acceptTypes;
    // |selectedFiles| has filenames which a file upload control already select.
    // A WebViewClient implementation may ask a user to select
    //  - removing a file from the selected files,
    //  - appending other files, or
    //  - replacing with other files
    // before opening a file chooser dialog.
    WebVector<WebString> selectedFiles;
};

} // namespace WebKit

#endif
