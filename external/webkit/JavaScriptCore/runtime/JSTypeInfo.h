
// -*- mode: c++; c-basic-offset: 4 -*-

#ifndef JSTypeInfo_h
#define JSTypeInfo_h

// This file would be called TypeInfo.h, but that conflicts with <typeinfo.h>
// in the STL on systems without case-sensitive file systems. 

#include "JSType.h"

namespace JSC {

    // WebCore uses MasqueradesAsUndefined to make document.all and style.filter undetectable.
    static const unsigned MasqueradesAsUndefined = 1;
    static const unsigned ImplementsHasInstance = 1 << 1;
    static const unsigned OverridesHasInstance = 1 << 2;
    static const unsigned ImplementsDefaultHasInstance = 1 << 3;
    static const unsigned NeedsThisConversion = 1 << 4;
    static const unsigned OverridesGetOwnPropertySlot = 1 << 5;
    static const unsigned OverridesMarkChildren = 1 << 6;
    static const unsigned OverridesGetPropertyNames = 1 << 7;

    class TypeInfo {
        friend class JIT;
    public:
        TypeInfo(JSType type, unsigned flags = 0)
            : m_type(type)
        {
            // ImplementsDefaultHasInstance means (ImplementsHasInstance & !OverridesHasInstance)
            if ((flags & (ImplementsHasInstance | OverridesHasInstance)) == ImplementsHasInstance)
                m_flags = flags | ImplementsDefaultHasInstance;
            else
                m_flags = flags;
        }

        JSType type() const { return m_type; }

        bool masqueradesAsUndefined() const { return m_flags & MasqueradesAsUndefined; }
        bool implementsHasInstance() const { return m_flags & ImplementsHasInstance; }
        bool overridesHasInstance() const { return m_flags & OverridesHasInstance; }
        bool needsThisConversion() const { return m_flags & NeedsThisConversion; }
        bool overridesGetOwnPropertySlot() const { return m_flags & OverridesGetOwnPropertySlot; }
        bool overridesMarkChildren() const { return m_flags & OverridesMarkChildren; }
        bool overridesGetPropertyNames() const { return m_flags & OverridesGetPropertyNames; }
        unsigned flags() const { return m_flags; }

    private:
        JSType m_type;
        unsigned m_flags;
    };

}

#endif // JSTypeInfo_h
