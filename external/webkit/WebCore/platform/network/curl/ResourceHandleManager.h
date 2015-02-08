

#ifndef ResourceHandleManager_h
#define ResourceHandleManager_h

#include "CString.h"
#include "Frame.h"
#include "PlatformString.h"
#include "Timer.h"
#include "ResourceHandleClient.h"

#if PLATFORM(WIN)
#include <winsock2.h>
#include <windows.h>
#endif

#include <curl/curl.h>
#include <wtf/Vector.h>

namespace WebCore {

class ResourceHandleManager {
public:
    enum ProxyType {
        HTTP = CURLPROXY_HTTP,
        Socks4 = CURLPROXY_SOCKS4,
        Socks4A = CURLPROXY_SOCKS4A,
        Socks5 = CURLPROXY_SOCKS5,
        Socks5Hostname = CURLPROXY_SOCKS5_HOSTNAME
    };
    static ResourceHandleManager* sharedInstance();
    void add(ResourceHandle*);
    void cancel(ResourceHandle*);
    void setCookieJarFileName(const char* cookieJarFileName);

    void dispatchSynchronousJob(ResourceHandle*);

    void setupPOST(ResourceHandle*, struct curl_slist**);
    void setupPUT(ResourceHandle*, struct curl_slist**);

    void setProxyInfo(const String& host = "",
                      unsigned long port = 0,
                      ProxyType type = HTTP,
                      const String& username = "",
                      const String& password = "");

private:
    ResourceHandleManager();
    ~ResourceHandleManager();
    void downloadTimerCallback(Timer<ResourceHandleManager>*);
    void removeFromCurl(ResourceHandle*);
    bool removeScheduledJob(ResourceHandle*);
    void startJob(ResourceHandle*);
    bool startScheduledJobs();

    void initializeHandle(ResourceHandle*);

    Timer<ResourceHandleManager> m_downloadTimer;
    CURLM* m_curlMultiHandle;
    CURLSH* m_curlShareHandle;
    char* m_cookieJarFileName;
    char m_curlErrorBuffer[CURL_ERROR_SIZE];
    Vector<ResourceHandle*> m_resourceHandleList;
    const CString m_certificatePath;
    int m_runningJobs;
    
    String m_proxy;
    ProxyType m_proxyType;
};

}

#endif
