

#include "config.h"
#include "ArchiveResourceCollection.h"

namespace WebCore {

ArchiveResourceCollection::ArchiveResourceCollection()
{
}

void ArchiveResourceCollection::addAllResources(Archive* archive)
{
    ASSERT(archive);
    if (!archive)
        return;
    
    const Vector<RefPtr<ArchiveResource> >& subresources = archive->subresources();
    Vector<RefPtr<ArchiveResource> >::const_iterator iRes = subresources.begin();
    Vector<RefPtr<ArchiveResource> >::const_iterator endRes = subresources.end();
    
    for (; iRes != endRes; ++iRes)
        m_subresources.set((*iRes)->url(), iRes->get());

    const Vector<RefPtr<Archive> >& subframes = archive->subframeArchives();
    Vector<RefPtr<Archive> >::const_iterator iFrame = subframes.begin();
    Vector<RefPtr<Archive> >::const_iterator endFrame = subframes.end();
        
    for (; iFrame != endFrame; ++iFrame) {        
        ASSERT((*iFrame)->mainResource());
        const String& frameName = (*iFrame)->mainResource()->frameName();
        if (!frameName.isNull())
            m_subframes.set(frameName, iFrame->get());
    }
}
    
// FIXME: Adding a resource directly to a DocumentLoader/ArchiveResourceCollection seems like bad design, but is API some apps rely on.
// Can we change the design in a manner that will let us deprecate that API without reducing functionality of those apps?
void ArchiveResourceCollection::addResource(PassRefPtr<ArchiveResource> resource)
{
    ASSERT(resource);
    if (!resource)
        return;

    const KURL& url = resource->url(); // get before passing PassRefPtr (which sets it to 0)
    m_subresources.set(url, resource);
}

ArchiveResource* ArchiveResourceCollection::archiveResourceForURL(const KURL& url)
{
    ArchiveResource* resource = m_subresources.get(url).get();
    if (!resource)
        return 0;
        
    return resource;
}

PassRefPtr<Archive> ArchiveResourceCollection::popSubframeArchive(const String& frameName)
{
    return m_subframes.take(frameName);
}

}
