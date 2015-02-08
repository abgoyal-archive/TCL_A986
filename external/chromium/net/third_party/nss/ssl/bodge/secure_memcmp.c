
// This file exists to provide the secure memcmp function. This was added in
// NSS 3.12.5.

#include <stdlib.h>

int
NSS_SecureMemcmp(const void *ia, const void *ib, size_t n)
{
    const unsigned char *a = (const unsigned char*) ia;
    const unsigned char *b = (const unsigned char*) ib;
    size_t i;
    unsigned char r = 0;

    for (i = 0; i < n; ++i) {
        r |= *a++ ^ *b++;
    }

    return r;
}
