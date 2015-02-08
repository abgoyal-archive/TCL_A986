

#ifndef WebDatabaseManager_h
#define WebDatabaseManager_h

#if ENABLE(DATABASE)

#include <WebCore/DatabaseTrackerClient.h>

#include "WebKit.h"

class WebDatabaseManager : public IWebDatabaseManager, private WebCore::DatabaseTrackerClient {
public:
    static WebDatabaseManager* createInstance();

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface( 
        /* [in] */ REFIID riid,
        /* [iid_is][out] */ void** ppvObject);

    virtual ULONG STDMETHODCALLTYPE AddRef();

    virtual ULONG STDMETHODCALLTYPE Release();

    // IWebDatabaseManager
    virtual HRESULT STDMETHODCALLTYPE sharedWebDatabaseManager( 
        /* [retval][out] */ IWebDatabaseManager** result);
        
    virtual HRESULT STDMETHODCALLTYPE origins( 
        /* [retval][out] */ IEnumVARIANT** result);
        
    virtual HRESULT STDMETHODCALLTYPE databasesWithOrigin( 
        /* [in] */ IWebSecurityOrigin* origin,
        /* [retval][out] */ IEnumVARIANT** result);
        
    virtual HRESULT STDMETHODCALLTYPE detailsForDatabase( 
        /* [in] */ BSTR databaseName,
        /* [in] */ IWebSecurityOrigin* origin,
        /* [retval][out] */ IPropertyBag** result);
        
    virtual HRESULT STDMETHODCALLTYPE deleteAllDatabases();
        
    virtual HRESULT STDMETHODCALLTYPE deleteOrigin( 
        /* [in] */ IWebSecurityOrigin* origin);
        
    virtual HRESULT STDMETHODCALLTYPE deleteDatabase( 
        /* [in] */ BSTR databaseName,
        /* [in] */ IWebSecurityOrigin* origin);

    virtual HRESULT STDMETHODCALLTYPE setQuota(
        /* [in] */ BSTR origin,
        /* [in] */ unsigned long long quota);

    // DatabaseTrackerClient
    virtual void dispatchDidModifyOrigin(WebCore::SecurityOrigin*);
    virtual void dispatchDidModifyDatabase(WebCore::SecurityOrigin*, const WebCore::String& databaseName);

private:
    WebDatabaseManager();
    ~WebDatabaseManager();

    ULONG m_refCount;
};

void WebKitSetWebDatabasesPathIfNecessary();

#endif

#endif
