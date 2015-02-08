

#include "config.h"
#include "IconRecord.h"

#include "BitmapImage.h"
#include "IconDatabase.h"
#include "Logging.h"
#include "SQLiteStatement.h"
#include "SQLiteTransaction.h"

#include <limits.h>

namespace WebCore {

IconRecord::IconRecord(const String& url)
    : m_iconURL(url)
    , m_stamp(0)
    , m_dataSet(false)
{

}

IconRecord::~IconRecord()
{
    LOG(IconDatabase, "Destroying IconRecord for icon url %s", m_iconURL.ascii().data());
}

Image* IconRecord::image(const IntSize&)
{
    // FIXME rdar://4680377 - For size right now, we are returning our one and only image and the Bridge
    // is resizing it in place.  We need to actually store all the original representations here and return a native
    // one, or resize the best one to the requested size and cache that result.
    
    return m_image.get();
}

void IconRecord::setImageData(PassRefPtr<SharedBuffer> data)
{
    // It's okay to delete the raw image here. Any existing clients using this icon will be
    // managing an image that was created with a copy of this raw image data.
    m_image = BitmapImage::create();

    // Copy the provided data into the buffer of the new Image object.
    if (!m_image->setData(data, true)) {
        LOG(IconDatabase, "Manual image data for iconURL '%s' FAILED - it was probably invalid image data", m_iconURL.ascii().data());
        m_image.clear();
    }
    
    m_dataSet = true;
}

void IconRecord::loadImageFromResource(const char* resource)
{
    if (!resource)
        return;
        
    m_image = Image::loadPlatformResource(resource);
    m_dataSet = true;
}

ImageDataStatus IconRecord::imageDataStatus()
{
    if (!m_dataSet)
        return ImageDataStatusUnknown;
    if (!m_image)
        return ImageDataStatusMissing;
    return ImageDataStatusPresent;
}

IconSnapshot IconRecord::snapshot(bool forDeletion) const
{
    if (forDeletion)
        return IconSnapshot(m_iconURL, 0, 0);
    
    return IconSnapshot(m_iconURL, m_stamp, m_image ? m_image->data() : 0);
}

} // namespace WebCore    
