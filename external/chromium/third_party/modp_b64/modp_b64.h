/* -*- mode: c; c-basic-offset: 4; indent-tabs-mode: nil; tab-width: 4 -*- */
/* vi: set expandtab shiftwidth=4 tabstop=4: */


#ifndef MODP_B64
#define MODP_B64

#ifdef __cplusplus
extern "C" {
#endif

int modp_b64_encode(char* dest, const char* str, int len);

int modp_b64_decode(char* dest, const char* src, int len);

#define modp_b64_encode_len(A) ((A+2)/3 * 4 + 1)

#define modp_b64_decode_len(A) (A / 4 * 3 + 2)

#define modp_b64_encode_strlen(A) ((A + 2)/ 3 * 4)

#ifdef __cplusplus
}

#include <string>

inline std::string& modp_b64_encode(std::string& s)
{
    std::string x(modp_b64_encode_len(s.size()), '\0');
    int d = modp_b64_encode(const_cast<char*>(x.data()), s.data(), s.size());
    x.erase(d, std::string::npos);
    s.swap(x);
    return s;
}

inline std::string& modp_b64_decode(std::string& s)
{
    std::string x(modp_b64_decode_len(s.size()), '\0');
    int d = modp_b64_decode(const_cast<char*>(x.data()), s.data(), s.size());
    if (d < 0) {
        x.clear();
    } else {
        x.erase(d, std::string::npos);
    }
    s.swap(x);
    return s;
}

#endif /* __cplusplus */

#endif /* MODP_B64 */
