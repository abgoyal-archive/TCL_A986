

#include "WebNetscapeDeprecatedFunctions.h"

#if ENABLE(NETSCAPE_PLUGIN_API) && !defined(__LP64__)

OSErr WebGetDiskFragment(const FSSpec *fileSpec, UInt32 offset, UInt32 length, ConstStr63Param fragName, CFragLoadOptions options, CFragConnectionID *connID, Ptr *mainAddr, Str255 errMessage)
{
    return GetDiskFragment(fileSpec, offset, length, fragName, options, connID, mainAddr, errMessage);
}

OSErr WebCloseConnection(CFragConnectionID *connID)
{
    return CloseConnection(connID);
}

SInt16 WebLMGetCurApRefNum(void)
{
    return LMGetCurApRefNum();
}

extern void WebLMSetCurApRefNum(SInt16 value)
{
    LMSetCurApRefNum(value);
}

#endif /* ENABLE(NETSCAPE_PLUGIN_API) && !defined(__LP64__) */
