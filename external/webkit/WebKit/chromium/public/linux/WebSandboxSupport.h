

#ifndef WebSandboxSupport_h
#define WebSandboxSupport_h

#include "../WebCommon.h"
#include "../WebString.h"

namespace WebKit {

// Put methods here that are required due to sandbox restrictions.
class WebSandboxSupport {
public:
    // Fonts ---------------------------------------------------------------

    // Get a font family which contains glyphs for the given Unicode
    // code-points.
    //   characters: a UTF-16 encoded string
    //   numCharacters: the number of 16-bit words in |characters|
    //
    // Returns a string with the font family on an empty string if the
    // request cannot be satisfied.
    virtual WebString getFontFamilyForCharacters(const WebUChar* characters, size_t numCharacters) = 0;
};

} // namespace WebKit

#endif
