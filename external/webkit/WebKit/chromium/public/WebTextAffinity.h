

#ifndef WebTextAffinity_h
#define WebTextAffinity_h

namespace WebKit {

// These constants specify the preferred direction of selection.
enum WebTextAffinity {
    WebTextAffinityUpstream,   // The selection is moving toward the top of the document.
    WebTextAffinityDownstream, // The selection is moving toward the bottom of the document.
};

} // namespace WebKit

#endif
