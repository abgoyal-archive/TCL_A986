

#include "config.h"

#if ENABLE(TOUCH_EVENTS)

#include "TouchList.h"

namespace WebCore {

Touch* TouchList::item(unsigned index)
{
    if (index >= m_values.size())
        return 0;
    return m_values[index].get();
}

} // namespace WebCore

#endif
