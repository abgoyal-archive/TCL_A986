

#ifndef DocumentMarker_h
#define DocumentMarker_h

#include "PlatformString.h"

namespace WebCore {
    class String;

// A range of a node within a document that is "marked", such as the range of a misspelled word.
// It optionally includes a description that could be displayed in the user interface.
// It also optionally includes a flag specifying whether the match is active, which is ignored
// for all types other than type TextMatch.
struct DocumentMarker {

    enum MarkerType {
        AllMarkers  = -1,
        Spelling,
        Grammar,
        TextMatch,
        Replacement
    };

    MarkerType type;
    unsigned startOffset;
    unsigned endOffset;
    String description;
    bool activeMatch;

    bool operator==(const DocumentMarker& o) const
    {
        return type == o.type && startOffset == o.startOffset && endOffset == o.endOffset;
    }

    bool operator!=(const DocumentMarker& o) const
    {
        return !(*this == o);
    }
};

} // namespace WebCore

#endif // DocumentMarker_h
