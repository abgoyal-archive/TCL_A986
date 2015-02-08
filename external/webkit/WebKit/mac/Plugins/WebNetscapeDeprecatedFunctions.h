

#if ENABLE(NETSCAPE_PLUGIN_API) && !defined(__LP64__)

#import <CoreServices/CoreServices.h>

#ifdef __cplusplus
extern "C" {
#endif

extern OSErr WebGetDiskFragment(const FSSpec *fileSpec, UInt32 offset, UInt32 length, ConstStr63Param fragName, CFragLoadOptions options, CFragConnectionID *connID, Ptr *mainAddr, Str255 errMessage);
extern OSErr WebCloseConnection(CFragConnectionID *connID);
extern SInt16 WebLMGetCurApRefNum(void);
extern void WebLMSetCurApRefNum(SInt16 value);

#ifdef __cplusplus
}
#endif
        
#endif /* ENABLE(NETSCAPE_PLUGIN_API) && !defined(__LP64__) */
