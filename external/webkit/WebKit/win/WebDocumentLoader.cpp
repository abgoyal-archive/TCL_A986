

#include "config.h"
#include "WebDocumentLoader.h"

#include "WebKitDLL.h"

using namespace WebCore;

WebDocumentLoader::WebDocumentLoader(const ResourceRequest& request, const SubstituteData& substituteData)
    : DocumentLoader(request, substituteData)
    , m_dataSource(0)
    , m_detachedDataSource(0)
{
    gClassCount++;
    gClassNameCount.add("WebDocumentLoader");
}

PassRefPtr<WebDocumentLoader> WebDocumentLoader::create(const ResourceRequest& req, const SubstituteData& data)
{
    return adoptRef(new WebDocumentLoader(req, data));
}

WebDocumentLoader::~WebDocumentLoader()
{
    gClassCount--;
    gClassNameCount.remove("WebDocumentLoader");
    if (m_dataSource) {
        ASSERT(!m_detachedDataSource);
        m_dataSource->Release();
    }
}

void WebDocumentLoader::setDataSource(WebDataSource *dataSource)
{
    ASSERT(!m_dataSource);
    m_dataSource = dataSource;
    if (m_dataSource)
        m_dataSource->AddRef();
}

WebDataSource* WebDocumentLoader::dataSource() const
{
    return m_dataSource;
}

void WebDocumentLoader::detachDataSource()
{
    // we only call detachDataSource when the WebDataSource is freed - and we won't get there if m_dataSource is not
    // null (because that would mean the loader still has a reference to the data source)
    ASSERT(!m_dataSource);
    m_detachedDataSource = 0;
}

void WebDocumentLoader::attachToFrame()
{
    DocumentLoader::attachToFrame();
    if (m_detachedDataSource) {
        ASSERT(!m_dataSource);
        setDataSource(m_detachedDataSource);
        m_detachedDataSource = 0;
    }
}

void WebDocumentLoader::detachFromFrame()
{
    DocumentLoader::detachFromFrame();
    m_detachedDataSource = m_dataSource;
    if (m_dataSource) {
        WebDataSource* dataSourceToBeReleased = m_dataSource;
        // It's important to null out m_dataSource before calling release on the data source.  That release can cause the data
        // source to be deleted - which ends up calling loader->detachDataSource() which makes the assumption that the loader no 
        // longer holds a reference to the data source.
        m_dataSource = 0;
        dataSourceToBeReleased->Release();
    }
}
