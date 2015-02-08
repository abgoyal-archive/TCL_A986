

#ifndef WebFontInfo_h
#define WebFontInfo_h

#include "../WebCString.h"

#include <string.h>
#include <unistd.h>

namespace WebKit {

class WebFontInfo {
public:
    // Return a font family which provides glyphs for the Unicode code points
    // specified by |utf16|
    //   characters: a native-endian UTF16 string
    //   numCharacters: the number of 16-bit words in |utf16|
    //
    // Returns: the font family or an empty string if the request could not be
    // satisfied.
    WEBKIT_API static WebCString familyForChars(const WebUChar* characters, size_t numCharacters);
};

} // namespace WebKit

#endif
