

#ifndef AnimationList_h
#define AnimationList_h

#include "Animation.h"
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class AnimationList : public FastAllocBase {
public:
    void fillUnsetProperties();
    bool operator==(const AnimationList& o) const;
    bool operator!=(const AnimationList& o) const
    {
        return !(*this == o);
    }
    
    size_t size() const { return m_animations.size(); }
    bool isEmpty() const { return m_animations.isEmpty(); }
    
    void resize(size_t n) { m_animations.resize(n); }
    void remove(size_t i) { m_animations.remove(i); }
    void append(PassRefPtr<Animation> anim) { m_animations.append(anim); }
    
    Animation* animation(size_t i) { return m_animations[i].get(); }
    const Animation* animation(size_t i) const { return m_animations[i].get(); }
    
private:
    Vector<RefPtr<Animation> > m_animations;
};    


} // namespace WebCore

#endif // AnimationList_h
