

#include "config.h"
#include "KeyframeList.h"
#include "RenderObject.h"

namespace WebCore {

KeyframeList::~KeyframeList()
{
    clear();
}

void KeyframeList::clear()
{
    m_keyframes.clear();
    m_properties.clear();
}

bool KeyframeList::operator==(const KeyframeList& o) const
{
    if (m_keyframes.size() != o.m_keyframes.size())
        return false;

    Vector<KeyframeValue>::const_iterator it2 = o.m_keyframes.begin();
    for (Vector<KeyframeValue>::const_iterator it1 = m_keyframes.begin(); it1 != m_keyframes.end(); ++it1) {
        if (it1->m_key != it2->m_key)
            return false;
        const RenderStyle& style1 = *it1->m_style;
        const RenderStyle& style2 = *it2->m_style;
        if (style1 != style2)
            return false;
        ++it2;
    }

    return true;
}

void KeyframeList::insert(float key, PassRefPtr<RenderStyle> style)
{
    if (key < 0 || key > 1)
        return;

    int index = -1;
    
    for (size_t i = 0; i < m_keyframes.size(); ++i) {
        if (m_keyframes[i].m_key == key) {
            index = (int) i;
            break;
        }
        if (m_keyframes[i].m_key > key) {
            // insert before
            m_keyframes.insert(i, KeyframeValue());
            index = (int) i;
            break;
        }
    }
    
    if (index < 0) {
        // append
        index = (int) m_keyframes.size();
        m_keyframes.append(KeyframeValue());
    }
    
    m_keyframes[index].m_key = key;
    m_keyframes[index].m_style = style;
}

} // namespace WebCore
