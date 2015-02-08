

#include "config.h"
#include "JSStringRef.h"

#include "InitializeThreading.h"
#include "OpaqueJSString.h"
#include <wtf/unicode/UTF8.h>

using namespace JSC;
using namespace WTF::Unicode;

JSStringRef JSStringCreateWithCharacters(const JSChar* chars, size_t numChars)
{
    initializeThreading();
    return OpaqueJSString::create(chars, numChars).releaseRef();
}

JSStringRef JSStringCreateWithUTF8CString(const char* string)
{
    initializeThreading();
    if (string) {
        size_t length = strlen(string);
        Vector<UChar, 1024> buffer(length);
        UChar* p = buffer.data();
        if (conversionOK == convertUTF8ToUTF16(&string, string + length, &p, p + length))
            return OpaqueJSString::create(buffer.data(), p - buffer.data()).releaseRef();
    }

    // Null string.
    return OpaqueJSString::create().releaseRef();
}

JSStringRef JSStringRetain(JSStringRef string)
{
    string->ref();
    return string;
}

void JSStringRelease(JSStringRef string)
{
    string->deref();
}

size_t JSStringGetLength(JSStringRef string)
{
    return string->length();
}

const JSChar* JSStringGetCharactersPtr(JSStringRef string)
{
    return string->characters();
}

size_t JSStringGetMaximumUTF8CStringSize(JSStringRef string)
{
    // Any UTF8 character > 3 bytes encodes as a UTF16 surrogate pair.
    return string->length() * 3 + 1; // + 1 for terminating '\0'
}

size_t JSStringGetUTF8CString(JSStringRef string, char* buffer, size_t bufferSize)
{
    if (!bufferSize)
        return 0;

    char* p = buffer;
    const UChar* d = string->characters();
    ConversionResult result = convertUTF16ToUTF8(&d, d + string->length(), &p, p + bufferSize - 1, true);
    *p++ = '\0';
    if (result != conversionOK && result != targetExhausted)
        return 0;

    return p - buffer;
}

bool JSStringIsEqual(JSStringRef a, JSStringRef b)
{
    unsigned len = a->length();
    return len == b->length() && 0 == memcmp(a->characters(), b->characters(), len * sizeof(UChar));
}

bool JSStringIsEqualToUTF8CString(JSStringRef a, const char* b)
{
    JSStringRef bBuf = JSStringCreateWithUTF8CString(b);
    bool result = JSStringIsEqual(a, bBuf);
    JSStringRelease(bBuf);
    
    return result;
}
