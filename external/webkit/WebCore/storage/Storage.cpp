

#include "config.h"
#include "Storage.h"

#if ENABLE(DOM_STORAGE)

#include "StorageArea.h"
#include "PlatformString.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

PassRefPtr<Storage> Storage::create(Frame* frame, PassRefPtr<StorageArea> storageArea)
{
    return adoptRef(new Storage(frame, storageArea));
}

Storage::Storage(Frame* frame, PassRefPtr<StorageArea> storageArea)
    : m_frame(frame)
    , m_storageArea(storageArea)
{
    ASSERT(m_frame);
    ASSERT(m_storageArea);
}

Storage::~Storage()
{
}

unsigned Storage::length() const
{
    if (!m_frame)
        return 0;

    return m_storageArea->length();
}

String Storage::key(unsigned index) const
{
    if (!m_frame)
        return String();

    return m_storageArea->key(index);
}

String Storage::getItem(const String& key) const
{
    if (!m_frame)
        return String();

    return m_storageArea->getItem(key);
}

void Storage::setItem(const String& key, const String& value, ExceptionCode& ec)
{
    ec = 0;
    if (!m_frame)
        return;

    m_storageArea->setItem(key, value, ec, m_frame);
}

void Storage::removeItem(const String& key)
{
    if (!m_frame)
        return;

    m_storageArea->removeItem(key, m_frame);
}

void Storage::clear()
{
    if (!m_frame)
        return;

    m_storageArea->clear(m_frame);
}

bool Storage::contains(const String& key) const
{
    if (!m_frame)
        return false;

    return m_storageArea->contains(key);
}

}

#endif // ENABLE(DOM_STORAGE)
