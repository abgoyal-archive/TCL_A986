

#ifndef CookieStorageWin_h
#define CookieStorageWin_h

typedef struct OpaqueCFHTTPCookieStorage*  CFHTTPCookieStorageRef;

namespace WebCore {

    CFHTTPCookieStorageRef currentCookieStorage();
    
    void setCurrentCookieStorage(CFHTTPCookieStorageRef cookieStorage);

}

#endif  // CookieStorageWin_h
