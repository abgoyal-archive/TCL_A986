

#include "config.h"
#include "DocumentLoaderGtk.h"

#include "webkitprivate.h"
#include "webkitwebdatasource.h"

using namespace WebCore;

namespace WebKit {

DocumentLoader::DocumentLoader(const ResourceRequest& request, const SubstituteData& substituteData)
    : WebCore::DocumentLoader(request, substituteData)
    , m_isDataSourceReffed(false)
    , m_dataSource(0)
{
}

void DocumentLoader::setDataSource(WebKitWebDataSource* dataSource)
{
    ASSERT(!m_dataSource);

    m_dataSource = dataSource;
    refDataSource();
}

void DocumentLoader::detachDataSource()
{
    unrefDataSource();
}

void DocumentLoader::attachToFrame()
{
    WebCore::DocumentLoader::attachToFrame();

    if (m_dataSource) {
        refDataSource();
        return;
    }

    // We may get to here without having a datasource, when the data
    // is coming from the page cache.
    WebKitWebDataSource* dataSource = webkit_web_data_source_new_with_loader(this);
    setDataSource(dataSource);
    g_object_unref(dataSource);
}

void DocumentLoader::detachFromFrame()
{
    WebCore::DocumentLoader::detachFromFrame();

    if (m_loadingResources.isEmpty())
        unrefDataSource();
}

void DocumentLoader::increaseLoadCount(unsigned long identifier)
{
    ASSERT(m_dataSource);

    if (m_loadingResources.contains(identifier))
        return;
    m_loadingResources.add(identifier);
    refDataSource();
}

void DocumentLoader::decreaseLoadCount(unsigned long identifier)
{
    HashSet<unsigned long>::iterator it = m_loadingResources.find(identifier);

    // It is valid for a load to be cancelled before it's started.
    if (it == m_loadingResources.end())
        return;

    m_loadingResources.remove(it);

    if (m_loadingResources.isEmpty() && !frame())
        unrefDataSource();
}

// helper methos to avoid ref count churn
void DocumentLoader::refDataSource()
{
    if (!m_dataSource || m_isDataSourceReffed)
        return;
    m_isDataSourceReffed = true;
    g_object_ref(m_dataSource);
}
void DocumentLoader::unrefDataSource()
{
    if (!m_isDataSourceReffed)
        return;
    ASSERT(m_dataSource);
    m_isDataSourceReffed = false;
    g_object_unref(m_dataSource);
    m_dataSource = 0;
}

} // end namespace WebKit
