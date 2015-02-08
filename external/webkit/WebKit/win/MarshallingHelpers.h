

#ifndef MarshallingHelpers_H
#define MarshallingHelpers_H

#include <CoreFoundation/CoreFoundation.h>

namespace WebCore {
    class IntRect;
    class KURL;
    class String;
}

class MarshallingHelpers
{
public:
    static WebCore::KURL BSTRToKURL(BSTR);
    static BSTR KURLToBSTR(const WebCore::KURL&);
    static CFURLRef PathStringToFileCFURLRef(const WebCore::String&);
    static WebCore::String FileCFURLRefToPathString(CFURLRef fileURL);
    static CFURLRef BSTRToCFURLRef(BSTR);
    static CFStringRef BSTRToCFStringRef(BSTR);
    static CFStringRef LPCOLESTRToCFStringRef(LPCOLESTR);
    static BSTR CFStringRefToBSTR(CFStringRef);
    static int CFNumberRefToInt(CFNumberRef);
    static CFNumberRef intToCFNumberRef(int);
    static CFAbsoluteTime DATEToCFAbsoluteTime(DATE);
    static DATE CFAbsoluteTimeToDATE(CFAbsoluteTime);
    static SAFEARRAY* stringArrayToSafeArray(CFArrayRef);
    static SAFEARRAY* intArrayToSafeArray(CFArrayRef);
    static SAFEARRAY* intRectToSafeArray(const WebCore::IntRect&);
    static SAFEARRAY* iunknownArrayToSafeArray(CFArrayRef);
    static CFArrayRef safeArrayToStringArray(SAFEARRAY*);
    static CFArrayRef safeArrayToIntArray(SAFEARRAY*);
    static CFArrayRef safeArrayToIUnknownArray(SAFEARRAY*);
    static const void* IUnknownRetainCallback(CFAllocatorRef, const void*);
    static void IUnknownReleaseCallback(CFAllocatorRef, const void*);
    static CFArrayCallBacks kIUnknownArrayCallBacks;
    static CFDictionaryValueCallBacks kIUnknownDictionaryValueCallBacks;

private:
    static CFAbsoluteTime MarshallingHelpers::windowsEpochAbsoluteTime();

private:
    MarshallingHelpers();
    ~MarshallingHelpers();
};

#endif
