

#ifndef AccessibilityList_h
#define AccessibilityList_h

#if PLATFORM(MAC) && (defined(BUILDING_ON_TIGER) || defined(BUILDING_ON_LEOPARD))
#define ACCESSIBILITY_LISTS 0
#else
#define ACCESSIBILITY_LISTS 1
#endif

#include "AccessibilityRenderObject.h"

namespace WebCore {
    
class AccessibilityList : public AccessibilityRenderObject {
    
private:
    AccessibilityList(RenderObject*);
public:
    static PassRefPtr<AccessibilityList> create(RenderObject*);
    virtual ~AccessibilityList();
    
    virtual bool isList() const { return true; }
    bool isUnorderedList() const;
    bool isOrderedList() const;
    bool isDefinitionList() const;

    virtual AccessibilityRole roleValue() const { return ListRole; }
    virtual bool accessibilityIsIgnored() const;
    
};
    
} // namespace WebCore

#endif // AccessibilityList_h
