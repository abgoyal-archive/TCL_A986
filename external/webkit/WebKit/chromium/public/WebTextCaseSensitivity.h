

#ifndef WebTextCaseSensitivity_h
#define WebTextCaseSensitivity_h

namespace WebKit {

// These constants specify the preferred case sensitivity.
enum WebTextCaseSensitivity {
    WebTextCaseSensitive,   // String matching is case-sensitive.
    WebTextCaseInsensitive, // String matching is case-insensitive.
};

} // namespace WebKit

#endif
