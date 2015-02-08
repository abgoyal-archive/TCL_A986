

#ifndef WebFindOptions_h
#define WebFindOptions_h

#include "WebString.h"

namespace WebKit {

// Options used when performing a find-in-page query.
struct WebFindOptions {
    // Whether to search forward or backward within the page.
    bool forward;

    // Whether search should be case-sensitive.
    bool matchCase;

    // Whether this operation is the first request or a follow-up.
    bool findNext;

    WebFindOptions()
        : forward(true)
        , matchCase(false)
        , findNext(false) { }
};

} // namespace WebKit

#endif
