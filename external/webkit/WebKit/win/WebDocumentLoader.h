

#include "WebDataSource.h"

#pragma warning(push, 0)
#include <WebCore/DocumentLoader.h>
#pragma warning(pop)

using namespace WebCore;

class WebDocumentLoader : public DocumentLoader
{
public:
    static PassRefPtr<WebDocumentLoader> create(const ResourceRequest&, const SubstituteData&);

    ~WebDocumentLoader();

    void setDataSource(WebDataSource*);
    WebDataSource* dataSource() const;
    void detachDataSource();

    virtual void attachToFrame();
    virtual void detachFromFrame();

private:
    WebDocumentLoader(const ResourceRequest&, const SubstituteData&);
    WebDataSource* m_dataSource;
    WebDataSource* m_detachedDataSource; // not retained
};
