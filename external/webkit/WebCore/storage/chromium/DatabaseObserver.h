

#ifndef DatabaseObserver_h
#define DatabaseObserver_h

namespace WebCore {

class Database;

// The implementation of this class is in the WebKit API (Chromium source tree)
// in webkit/api/src/DatabaseObserver.cpp.
class DatabaseObserver {
public:
    static void databaseOpened(Database*);
    static void databaseModified(Database*);
    static void databaseClosed(Database*);
};

}

#endif // DatabaseObserver_h
