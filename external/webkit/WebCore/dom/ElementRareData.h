

#ifndef ElementRareData_h
#define ElementRareData_h

#include "Element.h"
#include "NodeRareData.h"

namespace WebCore {

using namespace HTMLNames;

class ElementRareData : public NodeRareData {
public:
    ElementRareData();

    void resetComputedStyle();

    using NodeRareData::needsFocusAppearanceUpdateSoonAfterAttach;
    using NodeRareData::setNeedsFocusAppearanceUpdateSoonAfterAttach;

    IntSize m_minimumSizeForResizing;
    RefPtr<RenderStyle> m_computedStyle;
    QualifiedName m_idAttributeName;
};

inline IntSize defaultMinimumSizeForResizing()
{
    return IntSize(INT_MAX, INT_MAX);
}

inline ElementRareData::ElementRareData()
    : m_minimumSizeForResizing(defaultMinimumSizeForResizing())
    , m_idAttributeName(idAttr)
{
}

inline void ElementRareData::resetComputedStyle()
{
    m_computedStyle.clear();
}

}
#endif // ElementRareData_h
