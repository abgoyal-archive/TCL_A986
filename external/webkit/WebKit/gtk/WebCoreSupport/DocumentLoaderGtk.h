

#ifndef DocumentLoaderGtk_h
#define DocumentLoaderGtk_h

#include "DocumentLoader.h"
#include "webkitdefines.h"
#include "wtf/HashSet.h"

namespace WebCore {
    class ResourceRequest;
    class SubstituteData;
}

namespace WebKit {

class DocumentLoader : public WebCore::DocumentLoader {
public:
    static PassRefPtr<WebKit::DocumentLoader> create(const WebCore::ResourceRequest& request, const WebCore::SubstituteData& data)
    {
        return adoptRef(new DocumentLoader(request, data));
    }

    void setDataSource(WebKitWebDataSource*);
    void detachDataSource();
    WebKitWebDataSource* dataSource() const { return m_dataSource; }

    void increaseLoadCount(unsigned long identifier);
    void decreaseLoadCount(unsigned long identifier);

private:
    DocumentLoader(const WebCore::ResourceRequest&, const WebCore::SubstituteData&);

    virtual void attachToFrame();
    virtual void detachFromFrame();

    void refDataSource();
    void unrefDataSource();

    bool m_isDataSourceReffed;
    WebKitWebDataSource* m_dataSource;
    HashSet<unsigned long> m_loadingResources;
};

} // end namespace WebKit

#endif
