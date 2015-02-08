

#include "config.h"
#include "JSStringRefCF.h"

#include "APICast.h"
#include "InitializeThreading.h"
#include "JSStringRef.h"
#include "OpaqueJSString.h"
#include <runtime/UString.h>
#include <runtime/JSValue.h>
#include <wtf/OwnArrayPtr.h>

JSStringRef JSStringCreateWithCFString(CFStringRef string)
{
    JSC::initializeThreading();

    // We cannot use CFIndex here since CFStringGetLength can return values larger than
    // it can hold.  (<rdar://problem/6806478>)
    size_t length = CFStringGetLength(string);
    if (length) {
        OwnArrayPtr<UniChar> buffer(new UniChar[length]);
        CFStringGetCharacters(string, CFRangeMake(0, length), buffer.get());
        COMPILE_ASSERT(sizeof(UniChar) == sizeof(UChar), unichar_and_uchar_must_be_same_size);
        return OpaqueJSString::create(reinterpret_cast<UChar*>(buffer.get()), length).releaseRef();
    } else {
        return OpaqueJSString::create(0, 0).releaseRef();
    }
}

CFStringRef JSStringCopyCFString(CFAllocatorRef alloc, JSStringRef string)
{
    return CFStringCreateWithCharacters(alloc, reinterpret_cast<const UniChar*>(string->characters()), string->length());
}
