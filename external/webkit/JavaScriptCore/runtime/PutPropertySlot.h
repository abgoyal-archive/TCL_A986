
// -*- mode: c++; c-basic-offset: 4 -*-

#ifndef PutPropertySlot_h
#define PutPropertySlot_h

#include <wtf/Assertions.h>

namespace JSC {
    
    class JSObject;
    class JSFunction;
    
    class PutPropertySlot {
    public:
        enum Type { Uncachable, ExistingProperty, NewProperty };

        PutPropertySlot()
            : m_type(Uncachable)
            , m_base(0)
        {
        }

        void setExistingProperty(JSObject* base, size_t offset)
        {
            m_type = ExistingProperty;
            m_base = base;
            m_offset = offset;
        }

        void setNewProperty(JSObject* base, size_t offset)
        {
            m_type = NewProperty;
            m_base = base;
            m_offset = offset;
        }

        Type type() const { return m_type; }
        JSObject* base() const { return m_base; }

        bool isCacheable() const { return m_type != Uncachable; }
        size_t cachedOffset() const {
            ASSERT(isCacheable());
            return m_offset;
        }
    private:
        Type m_type;
        JSObject* m_base;
        size_t m_offset;
    };

} // namespace JSC

#endif // PutPropertySlot_h
