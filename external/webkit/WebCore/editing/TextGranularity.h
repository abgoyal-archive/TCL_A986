

#ifndef TextGranularity_h
#define TextGranularity_h

namespace WebCore {

// FIXME: This really should be broken up into more than one concept.
// Frame doesn't need the 3 boundaries in this enum.
enum TextGranularity {
    CharacterGranularity,
    WordGranularity,
    SentenceGranularity,
    LineGranularity,
    ParagraphGranularity,
    SentenceBoundary,
    LineBoundary,
    ParagraphBoundary,
    DocumentBoundary
};

}

#endif
