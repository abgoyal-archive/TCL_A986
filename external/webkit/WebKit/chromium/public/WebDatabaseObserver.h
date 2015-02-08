

#ifndef WebDatabaseObserver_h
#define WebDatabaseObserver_h

namespace WebKit {
class WebDatabase;

class WebDatabaseObserver {
public:
    virtual void databaseOpened(const WebDatabase&) = 0;
    virtual void databaseModified(const WebDatabase&) = 0;
    virtual void databaseClosed(const WebDatabase&) = 0;
protected:
    ~WebDatabaseObserver() {}
};

} // namespace WebKit

#endif
