

#include "config.h"
#include "ClientRectList.h"

#include "ExceptionCode.h"
#include "ClientRect.h"

namespace WebCore {

ClientRectList::ClientRectList()
{
}

ClientRectList::ClientRectList(const Vector<FloatQuad>& quads)
{
    m_list.reserveInitialCapacity(quads.size());
    for (size_t i = 0; i < quads.size(); ++i)
        m_list.append(ClientRect::create(quads[i].enclosingBoundingBox()));
}

ClientRectList::~ClientRectList()
{
}

unsigned ClientRectList::length() const
{
    return m_list.size();
}

ClientRect* ClientRectList::item(unsigned index)
{
    if (index >= m_list.size()) {
        // FIXME: this should throw an exception.
        // ec = INDEX_SIZE_ERR;
        return 0;
    }

    return m_list[index].get();
}

} // namespace WebCore
