
#ifndef DatabaseTrackerClient_h
#define DatabaseTrackerClient_h

#if ENABLE(DATABASE)

namespace WebCore {

class SecurityOrigin;
class String;

class DatabaseTrackerClient {
public:
    virtual ~DatabaseTrackerClient() { }
    virtual void dispatchDidModifyOrigin(SecurityOrigin*) = 0;
    virtual void dispatchDidModifyDatabase(SecurityOrigin*, const String& databaseName) = 0;
};

} // namespace WebCore

#endif

#endif // DatabaseTrackerClient_h
