

#if ENABLE(DATABASE) 

#import <WebCore/DatabaseTrackerClient.h>

class WebDatabaseTrackerClient : public WebCore::DatabaseTrackerClient {
public:
    static WebDatabaseTrackerClient* sharedWebDatabaseTrackerClient();
    
    virtual ~WebDatabaseTrackerClient();
    virtual void dispatchDidModifyOrigin(WebCore::SecurityOrigin*);
    virtual void dispatchDidModifyDatabase(WebCore::SecurityOrigin*, const WebCore::String& databaseIdentifier);
private:
    WebDatabaseTrackerClient();
};

#endif
