

#ifndef Base64_h
#define Base64_h

#include <wtf/Vector.h>

namespace WebCore {

void base64Encode(const Vector<char>&, Vector<char>&, bool insertLFs = false);
void base64Encode(const char*, unsigned, Vector<char>&, bool insertLFs = false);

// this decoder is not general purpose - it returns an error if it encounters a linefeed, as needed for window.atob
bool base64Decode(const Vector<char>&, Vector<char>&);
bool base64Decode(const char*, unsigned, Vector<char>&);

}

#endif // Base64_h
