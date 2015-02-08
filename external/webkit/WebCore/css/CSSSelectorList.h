

#ifndef CSSSelectorList_h
#define CSSSelectorList_h

#include "CSSSelector.h"
#include <wtf/Noncopyable.h>

namespace WebCore {
    
class CSSSelectorList : public Noncopyable {
public:
    CSSSelectorList() : m_selectorArray(0) { }
    ~CSSSelectorList();

    void adopt(CSSSelectorList& list);
    void adoptSelectorVector(Vector<CSSSelector*>& selectorVector);
    
    CSSSelector* first() const { return m_selectorArray ? m_selectorArray : 0; }
    static CSSSelector* next(CSSSelector* previous) { return previous->isLastInSelectorList() ? 0 : previous + 1; }
    bool hasOneSelector() const { return m_selectorArray ? m_selectorArray->isLastInSelectorList() : false; }

    bool selectorsNeedNamespaceResolution();

private:
    void deleteSelectors();

    // End of the array is indicated by m_isLastInSelectorList bit in the last item.
    CSSSelector* m_selectorArray;
};

} // namespace WebCore

#endif // CSSSelectorList_h
