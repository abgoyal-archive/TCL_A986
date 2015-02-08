

#include "config.h"
#include "StyleList.h"

#include <wtf/PassRefPtr.h>

namespace WebCore {

void StyleList::append(PassRefPtr<StyleBase> child)
{
    StyleBase* c = child.get();
    m_children.append(child);
    c->insertedIntoParent();
}

void StyleList::insert(unsigned position, PassRefPtr<StyleBase> child)
{
    StyleBase* c = child.get();
    if (position >= length())
        m_children.append(child);
    else
        m_children.insert(position, child);
    c->insertedIntoParent();
}

void StyleList::remove(unsigned position)
{
    if (position >= length())
        return;
    m_children.remove(position);
}

}
