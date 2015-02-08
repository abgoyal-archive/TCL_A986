

#ifndef WebKitSystemBits_h
#define WebKitSystemBits_h

typedef const struct __CFString* CFStringRef;

unsigned long long WebMemorySize();
unsigned long long WebVolumeFreeSize(CFStringRef path);

#endif // WebKitSystemBits_h
