

#ifndef NodeRareData_h
#define NodeRareData_h

#include "DynamicNodeList.h"
#include "EventListener.h"
#include "RegisteredEventListener.h"
#include "StringHash.h"
#include "QualifiedName.h"
#include <wtf/HashSet.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/OwnPtr.h>

namespace WebCore {

struct NodeListsNodeData : Noncopyable {
    typedef HashSet<DynamicNodeList*> NodeListSet;
    NodeListSet m_listsWithCaches;
    
    RefPtr<DynamicNodeList::Caches> m_childNodeListCaches;
    
    typedef HashMap<String, RefPtr<DynamicNodeList::Caches> > CacheMap;
    CacheMap m_classNodeListCaches;
    CacheMap m_nameNodeListCaches;
    
    typedef HashMap<QualifiedName, RefPtr<DynamicNodeList::Caches> > TagCacheMap;
    TagCacheMap m_tagNodeListCaches;

    static PassOwnPtr<NodeListsNodeData> create()
    {
        return new NodeListsNodeData;
    }
    
    void invalidateCaches();
    void invalidateCachesThatDependOnAttributes();
    bool isEmpty() const;

private:
    NodeListsNodeData()
        : m_childNodeListCaches(DynamicNodeList::Caches::create())
    {
    }
};
    
class NodeRareData : public Noncopyable {
public:    
    NodeRareData()
        : m_tabIndex(0)
        , m_tabIndexWasSetExplicitly(false)
        , m_isFocused(false)
        , m_needsFocusAppearanceUpdateSoonAfterAttach(false)
    {
    }

    typedef HashMap<const Node*, NodeRareData*> NodeRareDataMap;
    
    static NodeRareDataMap& rareDataMap()
    {
        static NodeRareDataMap* dataMap = new NodeRareDataMap;
        return *dataMap;
    }
    
    static NodeRareData* rareDataFromMap(const Node* node)
    {
        return rareDataMap().get(node);
    }
    
    void clearNodeLists() { m_nodeLists.clear(); }
    void setNodeLists(PassOwnPtr<NodeListsNodeData> lists) { m_nodeLists = lists; }
    NodeListsNodeData* nodeLists() const { return m_nodeLists.get(); }
    
    short tabIndex() const { return m_tabIndex; }
    void setTabIndexExplicitly(short index) { m_tabIndex = index; m_tabIndexWasSetExplicitly = true; }
    bool tabIndexSetExplicitly() const { return m_tabIndexWasSetExplicitly; }

    EventTargetData* eventTargetData() { return m_eventTargetData.get(); }
    EventTargetData* ensureEventTargetData()
    {
        if (!m_eventTargetData)
            m_eventTargetData.set(new EventTargetData);
        return m_eventTargetData.get();
    }

    bool isFocused() const { return m_isFocused; }
    void setFocused(bool focused) { m_isFocused = focused; }

protected:
    // for ElementRareData
    bool needsFocusAppearanceUpdateSoonAfterAttach() const { return m_needsFocusAppearanceUpdateSoonAfterAttach; }
    void setNeedsFocusAppearanceUpdateSoonAfterAttach(bool needs) { m_needsFocusAppearanceUpdateSoonAfterAttach = needs; }

private:
    OwnPtr<NodeListsNodeData> m_nodeLists;
    OwnPtr<EventTargetData> m_eventTargetData;
    short m_tabIndex;
    bool m_tabIndexWasSetExplicitly : 1;
    bool m_isFocused : 1;
    bool m_needsFocusAppearanceUpdateSoonAfterAttach : 1;
};

} //namespace

#endif
