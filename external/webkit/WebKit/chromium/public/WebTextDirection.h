

#ifndef WebTextDirection_h
#define WebTextDirection_h

namespace WebKit {

// Represents text directions (or writing directions) of a DOM node.
enum WebTextDirection {
    WebTextDirectionDefault,  // Natural writing direction ("inherit")
    WebTextDirectionLeftToRight,
    WebTextDirectionRightToLeft,
};

} // namespace WebKit

#endif
