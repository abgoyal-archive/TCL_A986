

#include "config.h"

#if ENABLE(SVG)
#include "SVGElementInstanceList.h"

namespace WebCore {

SVGElementInstanceList::SVGElementInstanceList(PassRefPtr<SVGElementInstance> rootInstance)
    : m_rootInstance(rootInstance)
{
}

SVGElementInstanceList::~SVGElementInstanceList()
{
}

unsigned SVGElementInstanceList::length() const
{
    // NOTE: We could use the same caching facilities, like the ones "ChildNodeList" uses.
    unsigned length = 0;
    for (SVGElementInstance* instance = m_rootInstance->firstChild(); instance; instance = instance->nextSibling())
        length++;
    return length;
}

SVGElementInstance* SVGElementInstanceList::item(unsigned index)
{
    unsigned pos = 0;
    SVGElementInstance* instance = m_rootInstance->firstChild();
    while (instance && pos < index) {
        instance = instance->nextSibling();
        pos++;
    }
    return instance;
}

}

#endif // ENABLE(SVG)

// vim:ts=4:noet
