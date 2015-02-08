

#include "config.h"
#include "MimeTypeArray.h"

#include "AtomicString.h"
#include "Frame.h"
#include "Page.h"
#include "Plugin.h"
#include "PluginData.h"

namespace WebCore {

MimeTypeArray::MimeTypeArray(Frame* frame)
    : m_frame(frame)
{
}

MimeTypeArray::~MimeTypeArray()
{
}

unsigned MimeTypeArray::length() const
{
    PluginData* data = getPluginData();
    if (!data)
        return 0;
    return data->mimes().size();
}

PassRefPtr<MimeType> MimeTypeArray::item(unsigned index)
{
    PluginData* data = getPluginData();
    if (!data)
        return 0;
    const Vector<MimeClassInfo*>& mimes = data->mimes();
    if (index >= mimes.size())
        return 0;
    return MimeType::create(data, index).get();
}

bool MimeTypeArray::canGetItemsForName(const AtomicString& propertyName)
{
    PluginData *data = getPluginData();
    if (!data)
        return 0;
    const Vector<MimeClassInfo*>& mimes = data->mimes();
    for (unsigned i = 0; i < mimes.size(); ++i) {
        if (mimes[i]->type == propertyName)
            return true;
    }
    return false;
}

PassRefPtr<MimeType> MimeTypeArray::namedItem(const AtomicString& propertyName)
{
    PluginData *data = getPluginData();
    if (!data)
        return 0;
    const Vector<MimeClassInfo*>& mimes = data->mimes();
    for (unsigned i = 0; i < mimes.size(); ++i) {
        if (mimes[i]->type == propertyName)
            return MimeType::create(data, i).get();
    }
    return 0;
}

PluginData* MimeTypeArray::getPluginData() const
{
    if (!m_frame)
        return 0;
    Page* p = m_frame->page();
    if (!p)
        return 0;
    return p->pluginData();
}

} // namespace WebCore
