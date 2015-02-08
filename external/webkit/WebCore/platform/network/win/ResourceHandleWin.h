

#ifndef ResourceHandleWin_h
#define ResourceHandleWin_h

#include <windows.h>

namespace WebCore {

struct PlatformDataStruct
{
    DWORD   error;
    BOOL    loaded;
    LPTSTR  errorString;
};

struct PlatformResponseStruct
{
};

}

#endif
