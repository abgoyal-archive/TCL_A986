

#include "config.h"
#include "PageURLRecord.h"

#include "IconRecord.h"

namespace WebCore {

PageURLRecord::PageURLRecord(const String& pageURL)
    : m_pageURL(pageURL)
    , m_retainCount(0)
{
}

PageURLRecord::~PageURLRecord()
{
    setIconRecord(0);
}

void PageURLRecord::setIconRecord(PassRefPtr<IconRecord> icon)
{
    if (m_iconRecord)
        m_iconRecord->m_retainingPageURLs.remove(m_pageURL);
        
    m_iconRecord = icon;
    
    if (m_iconRecord)
        m_iconRecord->m_retainingPageURLs.add(m_pageURL);
}
    
PageURLSnapshot PageURLRecord::snapshot(bool forDeletion) const 
{
    return PageURLSnapshot(m_pageURL, (m_iconRecord && !forDeletion) ? m_iconRecord->iconURL() : String());
}

} // namespace WebCore
