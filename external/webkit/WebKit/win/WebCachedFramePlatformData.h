
#ifndef WebCachedFramePlatformData_h
#define WebCachedFramePlatformData_h

#include "WebDataSource.h"

#include <WebCore/CachedFramePlatformData.h>

class WebCachedFramePlatformData : public WebCore::CachedFramePlatformData {
public:
    WebCachedFramePlatformData(IWebDataSource* webDataSource) : m_webDataSource(webDataSource) { }

    IWebDataSource* webDataSource() { return m_webDataSource.get(); }

private:
    COMPtr<IWebDataSource> m_webDataSource;
};

#endif // CachedFramePlatformData_h
