

#ifndef WebIconDatabase_H
#define WebIconDatabase_H

#include "WebKit.h"

#pragma warning(push, 0)
#include <WebCore/IconDatabaseClient.h>
#include <WebCore/IntSize.h>
#include <WebCore/IntSizeHash.h>
#pragma warning(pop)

#include <wtf/Vector.h>
#include <wtf/Threading.h>

#include <windows.h>

typedef const struct __CFString * CFStringRef;

namespace WebCore
{
    class IconDatabase;
}; //namespace WebCore
using namespace WebCore;
using namespace WTF;

class WebIconDatabase : public IWebIconDatabase, public WebCore::IconDatabaseClient
{
public:
    static WebIconDatabase* createInstance();
    static WebIconDatabase* sharedWebIconDatabase();
private:
    WebIconDatabase();
    ~WebIconDatabase();
    void init();
    void startUpIconDatabase();
    void shutDownIconDatabase();
public:

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // IWebIconDatabase
    virtual HRESULT STDMETHODCALLTYPE sharedIconDatabase( 
        /* [retval][out] */ IWebIconDatabase **result);
    
    virtual HRESULT STDMETHODCALLTYPE iconForURL( 
        /* [in] */ BSTR url,
        /* [optional][in] */ LPSIZE size,
        /* [optional][in] */ BOOL cache,
        /* [retval][out] */ OLE_HANDLE *image);
    
    virtual HRESULT STDMETHODCALLTYPE defaultIconWithSize( 
        /* [in] */ LPSIZE size,
        /* [retval][out] */ OLE_HANDLE *result);
    
    virtual HRESULT STDMETHODCALLTYPE retainIconForURL( 
        /* [in] */ BSTR url);
    
    virtual HRESULT STDMETHODCALLTYPE releaseIconForURL( 
        /* [in] */ BSTR url);

    virtual HRESULT STDMETHODCALLTYPE removeAllIcons( void);
    
    virtual HRESULT STDMETHODCALLTYPE delayDatabaseCleanup( void);
    
    virtual HRESULT STDMETHODCALLTYPE allowDatabaseCleanup( void);

    virtual HRESULT STDMETHODCALLTYPE iconURLForURL( 
        /* [in] */ BSTR url,
        /* [retval][out] */ BSTR *iconURL);

    virtual HRESULT STDMETHODCALLTYPE isEnabled( 
        /* [retval][out] */ BOOL *result);

    virtual HRESULT STDMETHODCALLTYPE setEnabled( 
        /* [in] */ BOOL /*flag*/);

    // IconDatabaseClient
    virtual void dispatchDidRemoveAllIcons();
    virtual void dispatchDidAddIconForPageURL(const WebCore::String&);

    static BSTR iconDatabaseDidAddIconNotification();
    static BSTR iconDatabaseDidRemoveAllIconsNotification();
    static CFStringRef iconDatabaseNotificationUserInfoURLKey();
protected:
    ULONG m_refCount;
    static WebIconDatabase* m_sharedWebIconDatabase;

    // Keep a set of HBITMAPs around for the default icon, and another
    // to share amongst present site icons
    HBITMAP getOrCreateSharedBitmap(LPSIZE size);
    HBITMAP getOrCreateDefaultIconBitmap(LPSIZE size);
    HashMap<IntSize, HBITMAP> m_defaultIconMap;
    HashMap<IntSize, HBITMAP> m_sharedIconMap;

    Mutex m_notificationMutex;
    Vector<String> m_notificationQueue;
    void scheduleNotificationDelivery();
    bool m_deliveryRequested;

    static void deliverNotifications(void*);
};

#endif
