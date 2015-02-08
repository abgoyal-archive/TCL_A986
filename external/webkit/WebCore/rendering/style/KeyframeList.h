

#ifndef KeyframeList_h
#define KeyframeList_h

#include "AtomicString.h"
#include <wtf/Vector.h>
#include <wtf/HashSet.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class RenderObject;
class RenderStyle;

class KeyframeValue {
public:
    KeyframeValue()
        : m_key(-1)
    {
    }

    float key() const { return m_key; }
    const RenderStyle* style() const { return m_style.get(); }

    float m_key;
    RefPtr<RenderStyle> m_style;
};

class KeyframeList {
public:
    KeyframeList(RenderObject* renderer, const AtomicString& animationName)
        : m_animationName(animationName)
        , m_renderer(renderer)
    {
        insert(0, 0);
        insert(1, 0);
    }
    ~KeyframeList();
        
    bool operator==(const KeyframeList& o) const;
    bool operator!=(const KeyframeList& o) const { return !(*this == o); }
    
    const AtomicString& animationName() const { return m_animationName; }
    
    void insert(float key, PassRefPtr<RenderStyle> style);
    
    void addProperty(int prop) { m_properties.add(prop); }
    bool containsProperty(int prop) const { return m_properties.contains(prop); }
    HashSet<int>::const_iterator beginProperties() const { return m_properties.begin(); }
    HashSet<int>::const_iterator endProperties() const { return m_properties.end(); }
    
    void clear();
    bool isEmpty() const { return m_keyframes.isEmpty(); }
    size_t size() const { return m_keyframes.size(); }
    Vector<KeyframeValue>::const_iterator beginKeyframes() const { return m_keyframes.begin(); }
    Vector<KeyframeValue>::const_iterator endKeyframes() const { return m_keyframes.end(); }

private:
    AtomicString m_animationName;
    Vector<KeyframeValue> m_keyframes;
    HashSet<int> m_properties;       // the properties being animated
    RenderObject* m_renderer;
};

} // namespace WebCore

#endif // KeyframeList_h
